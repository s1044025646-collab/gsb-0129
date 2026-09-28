import type { DatabaseSync } from "node:sqlite";
import { createHash, randomUUID } from "node:crypto";
import { Circle } from "./geometry.js";
import { ArmSpec, JointAngles, Pose } from "./kinematics.js";
import { PlannerParams, PlanResult } from "./planner.js";
import { Scenario } from "./scenario.js";

export interface ScenarioRecord extends Scenario {
  id: string;
  hash: string;
  createdAt: string;
  updatedAt: string;
}

export interface PlanRecord {
  id: string;
  clientRequestId: string | null;
  scenarioId: string;
  paramsVersion: string;
  scenarioHash: string;
  status: string;
  seed: number;
  iterationsUsed: number;
  budget: number;
  path: JointAngles[];
  trajectory: JointAngles[];
  report: PlanResult;
  inputSnapshot: {
    arm: ArmSpec;
    obstacles: Circle[];
    start: JointAngles;
    goal: JointAngles;
    goalPose: Pose;
    params: PlannerParams;
  };
  valid: boolean;
  invalidReason: string | null;
  createdAt: string;
}

export interface Page<T> {
  items: T[];
  total: number;
  limit: number;
  offset: number;
}

export type Clock = () => Date;

function sha256(value: unknown): string {
  return createHash("sha256").update(stableStringify(value)).digest("hex");
}

/** Key-ordered JSON so equal objects hash identically. */
export function stableStringify(value: unknown): string {
  if (value === null || typeof value !== "object") return JSON.stringify(value);
  if (Array.isArray(value)) return `[${value.map(stableStringify).join(",")}]`;
  const keys = Object.keys(value as Record<string, unknown>).sort();
  return `{${keys
    .map((k) => `${JSON.stringify(k)}:${stableStringify((value as Record<string, unknown>)[k])}`)
    .join(",")}}`;
}

export function computeScenarioHash(s: {
  arm: ArmSpec;
  obstacles: Circle[];
  start: JointAngles;
  goal: JointAngles;
  goalPose: Pose;
}): string {
  return sha256(s);
}

export class Store {
  constructor(
    private db: DatabaseSync,
    private clock: Clock = () => new Date()
  ) {}

  private now(): string {
    return this.clock().toISOString();
  }

  upsertScenario(input: {
    id?: string;
    name: string;
    description?: string;
    arm: ArmSpec;
    obstacles: Circle[];
    start: JointAngles;
    goal: JointAngles;
    goalPose: Pose;
  }): ScenarioRecord {
    const id = input.id ?? randomUUID();
    const ts = this.now();
    const hash = computeScenarioHash({
      arm: input.arm,
      obstacles: input.obstacles,
      start: input.start,
      goal: input.goal,
      goalPose: input.goalPose,
    });
    this.db
      .prepare(
        `INSERT INTO scenarios (id, name, description, arm_json, obstacles_json,
           start_json, goal_json, goal_pose_json, scenario_hash, created_at, updated_at)
         VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
         ON CONFLICT(id) DO UPDATE SET
           name=excluded.name, description=excluded.description,
           arm_json=excluded.arm_json, obstacles_json=excluded.obstacles_json,
           start_json=excluded.start_json, goal_json=excluded.goal_json,
           goal_pose_json=excluded.goal_pose_json, scenario_hash=excluded.scenario_hash,
           updated_at=excluded.updated_at`
      )
      .run(
        id,
        input.name,
        input.description ?? "",
        JSON.stringify(input.arm),
        JSON.stringify(input.obstacles),
        JSON.stringify(input.start),
        JSON.stringify(input.goal),
        JSON.stringify(input.goalPose),
        hash,
        ts,
        ts
      );

    // Existing plans for a changed scenario are invalidated.
    this.db
      .prepare(
        `UPDATE plans SET valid = 0, invalid_reason = 'SCENARIO_CHANGED'
         WHERE scenario_id = ? AND scenario_hash <> ? AND valid = 1`
      )
      .run(id, hash);

    return this.getScenario(id)!;
  }

  getScenario(id: string): ScenarioRecord | null {
    const row = this.db.prepare(`SELECT * FROM scenarios WHERE id = ?`).get(id) as
      | ScenarioRow
      | undefined;
    return row ? rowToScenario(row) : null;
  }

  listScenarios(limit: number, offset: number): Page<ScenarioRecord> {
    const total = (
      this.db.prepare(`SELECT COUNT(*) AS n FROM scenarios`).get() as { n: number }
    ).n;
    const rows = this.db
      .prepare(`SELECT * FROM scenarios ORDER BY created_at DESC, id LIMIT ? OFFSET ?`)
      .all(limit, offset) as unknown as ScenarioRow[];
    return { items: rows.map(rowToScenario), total, limit, offset };
  }

  putParamsVersion(version: string, params: PlannerParams): void {
    this.db
      .prepare(
        `INSERT INTO params_versions (version, params_json, created_at)
         VALUES (?, ?, ?)
         ON CONFLICT(version) DO UPDATE SET params_json = excluded.params_json`
      )
      .run(version, JSON.stringify(params), this.now());
  }

  getParamsVersion(version: string): PlannerParams | null {
    const row = this.db
      .prepare(`SELECT params_json FROM params_versions WHERE version = ?`)
      .get(version) as { params_json: string } | undefined;
    return row ? (JSON.parse(row.params_json) as PlannerParams) : null;
  }

  /** Idempotent: a repeated clientRequestId returns the original plan. */
  insertPlan(input: {
    clientRequestId?: string;
    scenarioId: string;
    result: PlanResult;
    params: PlannerParams;
    snapshot: PlanRecord["inputSnapshot"];
  }): { record: PlanRecord; reused: boolean } {
    if (input.clientRequestId) {
      const existing = this.db
        .prepare(`SELECT * FROM plans WHERE client_request_id = ?`)
        .get(input.clientRequestId) as PlanRow | undefined;
      if (existing) return { record: rowToPlan(existing), reused: true };
    }

    const scenario = this.getScenario(input.scenarioId);
    if (!scenario) throw new Error("SCENARIO_NOT_FOUND");
    const id = randomUUID();
    this.db
      .prepare(
        `INSERT INTO plans (id, client_request_id, scenario_id, params_version,
           scenario_hash, status, seed, iterations_used, budget, path_json,
           trajectory_json, report_json, input_snapshot_json, valid, invalid_reason, created_at)
         VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, 1, NULL, ?)`
      )
      .run(
        id,
        input.clientRequestId ?? null,
        input.scenarioId,
        input.params.paramsVersion,
        scenario.hash,
        input.result.status,
        input.result.seed,
        input.result.iterationsUsed,
        input.result.budget,
        JSON.stringify(input.result.path),
        JSON.stringify(input.result.trajectory),
        JSON.stringify(input.result),
        JSON.stringify(input.snapshot),
        this.now()
      );
    return { record: this.getPlan(id)!, reused: false };
  }

  getPlan(id: string): PlanRecord | null {
    const row = this.db.prepare(`SELECT * FROM plans WHERE id = ?`).get(id) as
      | PlanRow
      | undefined;
    return row ? rowToPlan(row) : null;
  }

  listPlans(scenarioId: string | null, limit: number, offset: number): Page<PlanRecord> {
    const where = scenarioId ? `WHERE scenario_id = ?` : ``;
    const args = scenarioId ? [scenarioId] : [];
    const total = (
      this.db
        .prepare(`SELECT COUNT(*) AS n FROM plans ${where}`)
        .get(...args) as { n: number }
    ).n;
    const rows = this.db
      .prepare(
        `SELECT * FROM plans ${where} ORDER BY created_at DESC, id LIMIT ? OFFSET ?`
      )
      .all(...args, limit, offset) as unknown as PlanRow[];
    return { items: rows.map(rowToPlan), total, limit, offset };
  }

  /** Recompute validity against the scenario hash stored at planning time. */
  refreshPlanValidity(id: string): PlanRecord | null {
    const plan = this.getPlan(id);
    if (!plan) return null;
    const scenario = this.getScenario(plan.scenarioId);
    if (scenario && scenario.hash !== plan.scenarioHash) {
      this.db
        .prepare(
          `UPDATE plans SET valid = 0, invalid_reason = 'SCENARIO_CHANGED'
           WHERE id = ? AND valid = 1`
        )
        .run(id);
    }
    return this.getPlan(id);
  }

  /** Test hook: run a callback inside a transaction and optionally roll back. */
  transaction<T>(fn: () => T, rollback = false): T {
    this.db.exec("BEGIN");
    try {
      const result = fn();
      if (rollback) this.db.exec("ROLLBACK");
      else this.db.exec("COMMIT");
      return result;
    } catch (err) {
      this.db.exec("ROLLBACK");
      throw err;
    }
  }
}

interface ScenarioRow {
  id: string;
  name: string;
  description: string;
  arm_json: string;
  obstacles_json: string;
  start_json: string;
  goal_json: string;
  goal_pose_json: string;
  scenario_hash: string;
  created_at: string;
  updated_at: string;
}

interface PlanRow {
  id: string;
  client_request_id: string | null;
  scenario_id: string;
  params_version: string;
  scenario_hash: string;
  status: string;
  seed: number;
  iterations_used: number;
  budget: number;
  path_json: string;
  trajectory_json: string;
  report_json: string;
  input_snapshot_json: string;
  valid: number;
  invalid_reason: string | null;
  created_at: string;
}

function rowToScenario(r: ScenarioRow): ScenarioRecord {
  return {
    id: r.id,
    name: r.name,
    description: r.description,
    arm: JSON.parse(r.arm_json) as ArmSpec,
    obstacles: JSON.parse(r.obstacles_json) as Circle[],
    start: JSON.parse(r.start_json) as JointAngles,
    goal: JSON.parse(r.goal_json) as JointAngles,
    goalPose: JSON.parse(r.goal_pose_json) as Pose,
    hash: r.scenario_hash,
    createdAt: r.created_at,
    updatedAt: r.updated_at,
  };
}

function rowToPlan(r: PlanRow): PlanRecord {
  return {
    id: r.id,
    clientRequestId: r.client_request_id,
    scenarioId: r.scenario_id,
    paramsVersion: r.params_version,
    scenarioHash: r.scenario_hash,
    status: r.status,
    seed: r.seed,
    iterationsUsed: r.iterations_used,
    budget: r.budget,
    path: JSON.parse(r.path_json) as JointAngles[],
    trajectory: JSON.parse(r.trajectory_json) as JointAngles[],
    report: JSON.parse(r.report_json) as PlanResult,
    inputSnapshot: JSON.parse(r.input_snapshot_json) as PlanRecord["inputSnapshot"],
    valid: r.valid === 1,
    invalidReason: r.invalid_reason,
    createdAt: r.created_at,
  };
}
