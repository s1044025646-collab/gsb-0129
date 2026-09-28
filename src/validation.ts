import { TAU } from "./geometry.js";
import { ArmSpec, JointAngles, Pose } from "./kinematics.js";
import { PlannerParams } from "./planner.js";

export class ValidationError extends Error {
  code = "VALIDATION_ERROR";
  details: Array<{ field: string; message: string }>;
  constructor(details: Array<{ field: string; message: string }>) {
    super(`Validation failed: ${details.map((d) => d.field).join(", ")}`);
    this.name = "ValidationError";
    this.details = details;
  }
}

type AnyObj = Record<string, unknown>;

function num(v: unknown, field: string, errs: Array<{ field: string; message: string }>): number {
  if (typeof v !== "number" || !Number.isFinite(v)) {
    errs.push({ field, message: "must be a finite number" });
    return NaN;
  }
  return v;
}

export function validateArm(raw: unknown): ArmSpec {
  const errs: Array<{ field: string; message: string }> = [];
  if (typeof raw !== "object" || raw === null) {
    throw new ValidationError([{ field: "arm", message: "must be an object" }]);
  }
  const o = raw as AnyObj;
  const arm: ArmSpec = {
    baseX: num(o.baseX, "arm.baseX", errs),
    baseY: num(o.baseY, "arm.baseY", errs),
    link1: num(o.link1, "arm.link1", errs),
    link2: num(o.link2, "arm.link2", errs),
    theta1Min: num(o.theta1Min, "arm.theta1Min", errs),
    theta1Max: num(o.theta1Max, "arm.theta1Max", errs),
    theta2Min: num(o.theta2Min, "arm.theta2Min", errs),
    theta2Max: num(o.theta2Max, "arm.theta2Max", errs),
  };
  if (arm.link1 <= 0) errs.push({ field: "arm.link1", message: "must be > 0" });
  if (arm.link2 <= 0) errs.push({ field: "arm.link2", message: "must be > 0" });
  for (const [lo, hi, name] of [
    [arm.theta1Min, arm.theta1Max, "theta1"],
    [arm.theta2Min, arm.theta2Max, "theta2"],
  ] as const) {
    if (Number.isFinite(lo) && Number.isFinite(hi)) {
      if (hi < lo) errs.push({ field: `arm.${name}`, message: "max < min" });
      if (hi - lo > TAU)
        errs.push({ field: `arm.${name}`, message: "limit interval wider than 2*PI" });
    }
  }
  if (errs.length) throw new ValidationError(errs);
  return arm;
}

export function validateObstacles(raw: unknown) {
  const errs: Array<{ field: string; message: string }> = [];
  if (!Array.isArray(raw)) {
    throw new ValidationError([{ field: "obstacles", message: "must be an array" }]);
  }
  const ids = new Set<string>();
  const list = raw.map((item, i) => {
    const field = `obstacles[${i}]`;
    if (typeof item !== "object" || item === null) {
      errs.push({ field, message: "must be an object" });
      return { id: "", x: 0, y: 0, radius: NaN };
    }
    const o = item as AnyObj;
    const id = typeof o.id === "string" && o.id ? o.id : `obs${i}`;
    if (ids.has(id)) errs.push({ field: `${field}.id`, message: "duplicate id" });
    ids.add(id);
    const radius = num(o.radius, `${field}.radius`, errs);
    if (Number.isFinite(radius) && radius <= 0)
      errs.push({ field: `${field}.radius`, message: "must be > 0" });
    return { id, x: num(o.x, `${field}.x`, errs), y: num(o.y, `${field}.y`, errs), radius };
  });
  if (errs.length) throw new ValidationError(errs);
  return list;
}

export function validateJoints(raw: unknown, field: string): JointAngles {
  const errs: Array<{ field: string; message: string }> = [];
  if (typeof raw !== "object" || raw === null) {
    throw new ValidationError([{ field, message: "must be an object" }]);
  }
  const o = raw as AnyObj;
  const q: JointAngles = {
    theta1: num(o.theta1, `${field}.theta1`, errs),
    theta2: num(o.theta2, `${field}.theta2`, errs),
  };
  if (errs.length) throw new ValidationError(errs);
  return q;
}

export function validatePose(raw: unknown): Pose {
  if (typeof raw !== "object" || raw === null) {
    throw new ValidationError([{ field: "goalPose", message: "must be an object" }]);
  }
  const o = raw as AnyObj;
  const errs: Array<{ field: string; message: string }> = [];
  const pose: Pose = {
    x: num(o.x, "goalPose.x", errs),
    y: num(o.y, "goalPose.y", errs),
    phi: o.phi === null || o.phi === undefined ? null : num(o.phi, "goalPose.phi", errs),
  };
  if (errs.length) throw new ValidationError(errs);
  return pose;
}

export function validateParams(raw: Partial<PlannerParams>): PlannerParams {
  const errs: Array<{ field: string; message: string }> = [];
  const p = { ...requireDefaultish(raw) };
  if (!Number.isInteger(p.seed)) errs.push({ field: "seed", message: "must be an integer" });
  if (p.maxIterations <= 0) errs.push({ field: "maxIterations", message: "must be > 0" });
  if (p.step <= 0) errs.push({ field: "step", message: "must be > 0" });
  if (p.goalBias < 0 || p.goalBias > 1)
    errs.push({ field: "goalBias", message: "must be in [0,1]" });
  if (p.safetyMargin < 0) errs.push({ field: "safetyMargin", message: "must be >= 0" });
  if (p.maxCartesianStep <= 0)
    errs.push({ field: "maxCartesianStep", message: "must be > 0" });
  if (typeof p.paramsVersion !== "string" || !p.paramsVersion)
    errs.push({ field: "paramsVersion", message: "must be a non-empty string" });
  if (errs.length) throw new ValidationError(errs);
  return p;
}

function requireDefaultish(raw: Partial<PlannerParams>): PlannerParams {
  // Filled with defaults so callers may send partial bodies.
  return {
    seed: typeof raw.seed === "number" ? raw.seed : 12345,
    maxIterations: raw.maxIterations ?? 2000,
    step: raw.step ?? 0.25,
    goalBias: raw.goalBias ?? 0.12,
    safetyMargin: raw.safetyMargin ?? 0.01,
    maxCartesianStep: raw.maxCartesianStep ?? 0.04,
    paramsVersion: raw.paramsVersion ?? "v1",
  };
}

export function validatePagination(query: URLSearchParams): { limit: number; offset: number } {
  const limitRaw = Number(query.get("limit") ?? "50");
  const offsetRaw = Number(query.get("offset") ?? "0");
  if (!Number.isInteger(limitRaw) || limitRaw < 1 || limitRaw > 200) {
    throw new ValidationError([
      { field: "limit", message: "must be an integer in [1,200]" },
    ]);
  }
  if (!Number.isInteger(offsetRaw) || offsetRaw < 0) {
    throw new ValidationError([{ field: "offset", message: "must be a non-negative integer" }]);
  }
  return { limit: limitRaw, offset: offsetRaw };
}
