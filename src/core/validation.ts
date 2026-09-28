import type { PlanningParams, ScenarioSpec } from "./types.js";

export interface FieldError {
  path: string;
  message: string;
}

const isNum = (v: unknown): v is number =>
  typeof v === "number" && Number.isFinite(v);

export function validateScenario(input: unknown): { ok: true; value: ScenarioSpec } | { ok: false; errors: FieldError[] } {
  const errors: FieldError[] = [];
  if (typeof input !== "object" || input === null) {
    return { ok: false, errors: [{ path: "$", message: "must be an object" }] };
  }
  const s = input as Record<string, unknown>;
  if (typeof s.name !== "string" || s.name.trim() === "") {
    errors.push({ path: "name", message: "must be a non-empty string" });
  }
  const arm = s.arm as Record<string, unknown> | undefined;
  if (!arm || typeof arm !== "object") {
    errors.push({ path: "arm", message: "required object" });
  } else {
    if (!isNum(arm.l1) || arm.l1 <= 0) errors.push({ path: "arm.l1", message: "must be > 0" });
    if (!isNum(arm.l2) || arm.l2 <= 0) errors.push({ path: "arm.l2", message: "must be > 0" });
    const base = arm.base as Record<string, unknown> | undefined;
    if (!base || !isNum(base.x) || !isNum(base.y)) {
      errors.push({ path: "arm.base", message: "requires numeric x,y" });
    }
    const lim = arm.limits as Record<string, unknown> | undefined;
    if (!lim) {
      errors.push({ path: "arm.limits", message: "required" });
    } else {
      const keys = ["theta1Min", "theta1Max", "theta2Min", "theta2Max"] as const;
      for (const k of keys) {
        if (!isNum(lim[k])) errors.push({ path: `arm.limits.${k}`, message: "must be a number" });
      }
      if (
        isNum(lim.theta1Min) && isNum(lim.theta1Max) &&
        lim.theta1Max < lim.theta1Min
      ) errors.push({ path: "arm.limits.theta1Max", message: "must be >= theta1Min" });
      if (
        isNum(lim.theta2Min) && isNum(lim.theta2Max) &&
        lim.theta2Max < lim.theta2Min
      ) errors.push({ path: "arm.limits.theta2Max", message: "must be >= theta2Min" });
    }
  }
  if (!Array.isArray(s.obstacles)) {
    errors.push({ path: "obstacles", message: "must be an array" });
  } else {
    s.obstacles.forEach((o, i) => {
      const ob = o as Record<string, unknown>;
      if (typeof ob.id !== "string" || ob.id === "") errors.push({ path: `obstacles[${i}].id`, message: "required string" });
      if (!isNum(ob.x) || !isNum(ob.y)) errors.push({ path: `obstacles[${i}]`, message: "x,y must be numbers" });
      if (!isNum(ob.radius) || ob.radius <= 0) errors.push({ path: `obstacles[${i}].radius`, message: "must be > 0" });
    });
  }
  for (const key of ["start", "goal"] as const) {
    const p = s[key] as Record<string, unknown> | undefined;
    if (!p || !isNum(p.x) || !isNum(p.y) || !isNum(p.phi)) {
      errors.push({ path: key, message: "requires numeric x,y,phi" });
    }
  }
  if (errors.length) return { ok: false, errors };
  return { ok: true, value: input as ScenarioSpec };
}

export function validateParams(input: unknown): { ok: true; value: PlanningParams } | { ok: false; errors: FieldError[] } {
  const errors: FieldError[] = [];
  if (typeof input !== "object" || input === null) {
    return { ok: false, errors: [{ path: "$", message: "must be an object" }] };
  }
  const p = input as Record<string, unknown>;
  if (typeof p.version !== "string" || p.version.trim() === "") {
    errors.push({ path: "version", message: "must be a non-empty string" });
  }
  const numChecks: [string, number, number][] = [
    ["step", 1e-6, Math.PI],
    ["resolution", 1e-6, Math.PI],
    ["maxIterations", 1, 2_000_000],
    ["goalBias", 0, 1],
  ];
  for (const [k, lo, hi] of numChecks) {
    if (!isNum(p[k]) || (p[k] as number) < lo || (p[k] as number) > hi) {
      errors.push({ path: k, message: `must be number in [${lo}, ${hi}]` });
    }
  }
  if (!isNum(p.seed)) errors.push({ path: "seed", message: "must be a finite number" });
  if (isNum(p.step) && isNum(p.resolution) && (p.resolution as number) > (p.step as number)) {
    errors.push({ path: "resolution", message: "must be <= step to avoid tunneling" });
  }
  if (errors.length) return { ok: false, errors };
  return { ok: true, value: input as PlanningParams };
}

export function defaultParams(seed = 12345): PlanningParams {
  return {
    version: "v1",
    step: 0.25,
    resolution: 0.02,
    maxIterations: 4000,
    goalBias: 0.12,
    seed,
  };
}
