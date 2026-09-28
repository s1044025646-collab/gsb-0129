import type { DatabaseSync } from "node:sqlite";
import { configCollision, validatePath } from "../core/collision.js";
import { forwardKinematics, inverseKinematics, solvePose } from "../core/kinematics.js";
import { planPath, PlannerError } from "../core/planner.js";
import { exportTrajectory, toCsv } from "../core/trajectory.js";
import {
  planRepository,
  scenarioRepository,
  type PlanRow,
  type ScenarioRow,
} from "../db/database.js";
import type { PlanningParams, ScenarioSpec } from "../core/types.js";

export interface Clock {
  now(): number;
  iso(): string;
}

export const systemClock: Clock = {
  now: () => Date.now(),
  iso: () => new Date().toISOString(),
};

export function parseScenario(row: ScenarioRow): ScenarioSpec {
  return JSON.parse(row.spec) as ScenarioSpec;
}

export function parsePlan(row: PlanRow) {
  return {
    id: row.id,
    scenarioId: row.scenario_id,
    idempotencyKey: row.idempotency_key,
    params: JSON.parse(row.params) as PlanningParams,
    configStart: JSON.parse(row.config_start),
    configGoal: JSON.parse(row.config_goal),
    path: JSON.parse(row.path),
    report: JSON.parse(row.report),
    valid: row.valid === 1,
    createdAt: row.created_at,
    invalidatedAt: row.invalidated_at,
  };
}

export class PlanningService {
  scenarios;
  plans;
  constructor(private db: DatabaseSync, private clock: Clock = systemClock) {
    this.scenarios = scenarioRepository(db);
    this.plans = planRepository(db);
  }

 createScenario(name: string, spec: ScenarioSpec): ScenarioRow {
    return this.scenarios.create({ name, spec, now: () => this.clock.iso() });
  }

  updateScenario(id: string, patch: { name?: string; spec?: ScenarioSpec }): ScenarioRow | undefined {
    return this.scenarios.update(id, { ...patch, now: () => this.clock.iso() });
  }

  fk(scenarioId: string, theta1: number, theta2: number) {
    const row = this.scenarios.get(scenarioId);
    if (!row) throw new ServiceError("NOT_FOUND", "scenario not found");
    const spec = parseScenario(row);
    return forwardKinematics(spec.arm, { theta1, theta2 });
  }

  ik(scenarioId: string, x: number, y: number) {
    const row = this.scenarios.get(scenarioId);
    if (!row) throw new ServiceError("NOT_FOUND", "scenario not found");
    const spec = parseScenario(row);
    try {
      const raw = inverseKinematics(spec.arm, { x, y });
      return raw.map((s) => ({
        ...s,
        fk: forwardKinematics(spec.arm, s),
        inLimits:
          s.theta1 >= spec.arm.limits.theta1Min && s.theta1 <= spec.arm.limits.theta1Max &&
          s.theta2 >= spec.arm.limits.theta2Min && s.theta2 <= spec.arm.limits.theta2Max,
      }));
    } catch (e) {
      throw new ServiceError((e as { code?: string }).code ?? "IK_ERROR", (e as Error).message);
    }
  }

  /**
   * Plan and persist. The stored record contains a full input snapshot
   * (params, chosen start/goal configs), so later scenario edits can flag
   * the plan invalid without mutating its snapshot.
   */
  plan(scenarioId: string, params: PlanningParams, idempotencyKey?: string) {
    const row = this.scenarios.get(scenarioId);
    if (!row) throw new ServiceError("NOT_FOUND", "scenario not found");
    const spec = parseScenario(row);
    let result;
    try {
      result = planPath(
        spec.arm,
        spec.obstacles,
        spec.start,
        spec.goal,
        params,
        { now: () => this.clock.now() },
      );
    } catch (e) {
      if (e instanceof PlannerError) throw new ServiceError(e.code, e.message);
      throw e;
    }
    const { row: planRow, reused } = this.plans.create({
      scenarioId,
      idempotencyKey,
      params,
      configStart: result.start,
      configGoal: result.goal,
      path: result.path,
      report: result.report,
      now: () => this.clock.iso(),
    });
    return { plan: parsePlan(planRow), reused };
  }

  /** Re-validate every interpolated segment of a stored plan against the
   * CURRENT scenario spec. A previously valid plan may now fail after edits. */
  verify(planId: string, resolution?: number) {
    const planRow = this.plans.get(planId);
    if (!planRow) throw new ServiceError("NOT_FOUND", "plan not found");
    const plan = parsePlan(planRow);
    const scRow = this.scenarios.get(plan.scenarioId);
    if (!scRow) throw new ServiceError("NOT_FOUND", "scenario not found");
    const spec = parseScenario(scRow);
    const res = resolution ?? plan.params.resolution;
    const validation = validatePath(spec.arm, spec.obstacles, plan.path, res);
    const startHit = plan.path.length
      ? configCollision(spec.arm, spec.obstacles, plan.path[0]!)
      : null;
    const goalHit = plan.path.length
      ? configCollision(spec.arm, spec.obstacles, plan.path[plan.path.length - 1]!)
      : null;
    return {
      planId,
      storedValid: plan.valid,
      specVersion: scRow.spec_version,
      validation,
      startHit,
      goalHit,
    };
  }

  trajectory(planId: string, format: "json" | "csv" = "json") {
    const planRow = this.plans.get(planId);
    if (!planRow) throw new ServiceError("NOT_FOUND", "plan not found");
    const scRow = this.scenarios.get(planRow.scenario_id);
    if (!scRow) throw new ServiceError("NOT_FOUND", "scenario not found");
    const spec = parseScenario(scRow);
    const plan = parsePlan(planRow);
    const rows = exportTrajectory(spec.arm, plan.path);
    return format === "csv" ? toCsv(rows) : rows;
  }

  /**
   * Run the same scenario with two parameter versions and return a comparison.
   * Does NOT persist anything (planning is deterministic); useful for tuning.
   */
  compare(scenarioId: string, a: PlanningParams, b: PlanningParams) {
    const row = this.scenarios.get(scenarioId);
    if (!row) throw new ServiceError("NOT_FOUND", "scenario not found");
    const spec = parseScenario(row);
    const run = (p: PlanningParams) => {
      try {
        return planPath(spec.arm, spec.obstacles, spec.start, spec.goal, p, {
          now: () => this.clock.now(),
        });
      } catch (e) {
        if (e instanceof PlannerError) {
          return { status: "error" as const, code: e.code, path: [] as never[], report: undefined };
        }
        throw e;
      }
    };
    const ra = run(a);
    const rb = run(b);
    return {
      a: { params: a, status: ra.status, report: ra.report, length: ra.path.length },
      b: { params: b, status: rb.status, report: rb.report, length: rb.path.length },
      sameStatus: ra.status === rb.status,
      samePathLength: ra.path.length === rb.path.length,
    };
  }

  solveStartGoal(scenarioId: string) {
    const row = this.scenarios.get(scenarioId);
    if (!row) throw new ServiceError("NOT_FOUND", "scenario not found");
    const spec = parseScenario(row);
    return {
      start: solvePose(spec.arm, spec.start),
      goal: solvePose(spec.arm, spec.goal),
    };
  }
}

export class ServiceError extends Error {
  code: string;
  constructor(code: string, message: string) {
    super(message);
    this.name = "ServiceError";
    this.code = code;
  }
}
