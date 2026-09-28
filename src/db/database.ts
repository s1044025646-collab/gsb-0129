import { DatabaseSync } from "node:sqlite";
import { randomUUID } from "node:crypto";
import { mkdirSync } from "node:fs";
import { dirname } from "node:path";

export interface ScenarioRow {
  id: string;
  name: string;
  spec: string;
  spec_version: number;
  created_at: string;
  updated_at: string;
}

export interface PlanRow {
  id: string;
  scenario_id: string;
  idempotency_key: string | null;
  params: string;
  config_start: string;
  config_goal: string;
  path: string;
  report: string;
  valid: number;
  created_at: string;
  invalidated_at: string | null;
}

/**
 * Open/creates the SQLite database and runs idempotent migrations.
 * Migrations use IF NOT EXISTS and a user_version pragma so they can be
 * safely re-executed on every startup.
 */
export function openDatabase(filePath: string): DatabaseSync {
  mkdirSync(dirname(filePath), { recursive: true });
  const db = new DatabaseSync(filePath);
  db.exec("PRAGMA journal_mode = WAL;");
  db.exec("PRAGMA foreign_keys = ON;");
  migrate(db);
  return db;
}

export function migrate(db: DatabaseSync): void {
  db.exec(`
    CREATE TABLE IF NOT EXISTS scenarios (
      id TEXT PRIMARY KEY,
      name TEXT NOT NULL,
      spec TEXT NOT NULL,
      spec_version INTEGER NOT NULL DEFAULT 1,
      created_at TEXT NOT NULL,
      updated_at TEXT NOT NULL
    );

    CREATE TABLE IF NOT EXISTS plans (
      id TEXT PRIMARY KEY,
      scenario_id TEXT NOT NULL REFERENCES scenarios(id),
      idempotency_key TEXT,
      params TEXT NOT NULL,
      config_start TEXT NOT NULL,
      config_goal TEXT NOT NULL,
      path TEXT NOT NULL,
      report TEXT NOT NULL,
      valid INTEGER NOT NULL DEFAULT 1,
      created_at TEXT NOT NULL,
      invalidated_at TEXT
    );

    CREATE INDEX IF NOT EXISTS idx_plans_scenario ON plans(scenario_id);
    CREATE UNIQUE INDEX IF NOT EXISTS idx_plans_idem
      ON plans(idempotency_key) WHERE idempotency_key IS NOT NULL;

    CREATE TABLE IF NOT EXISTS meta (
      key TEXT PRIMARY KEY,
      value TEXT NOT NULL
    );
  `);
  db.exec("PRAGMA user_version = 1;");
}

export interface ListPage<T> {
  items: T[];
  total: number;
  limit: number;
  offset: number;
}

export function parsePaging(query: { limit?: unknown; offset?: unknown }): { limit: number; offset: number } {
  const limit = Math.min(200, Math.max(1, Number(query.limit ?? 50) || 50));
  const offset = Math.max(0, Number(query.offset ?? 0) || 0);
  return { limit, offset };
}

export interface ScenarioRepository {
  create(input: { name: string; spec: unknown; now?: () => string }): ScenarioRow;
  get(id: string): ScenarioRow | undefined;
  list(limit: number, offset: number): ListPage<ScenarioRow>;
  /**
   * Update a scenario spec. Any previously generated plans for it are marked
   * invalid in the SAME transaction, so obstacle/size changes never leave
   * stale paths appearing valid.
   */
  update(id: string, input: { name?: string; spec?: unknown; now?: () => string }): ScenarioRow | undefined;
  invalidatePlans(scenarioId: string, nowIso: string): void;
}

export function scenarioRepository(db: DatabaseSync): ScenarioRepository {
  const insert = db.prepare(
    `INSERT INTO scenarios (id, name, spec, spec_version, created_at, updated_at)
     VALUES (?, ?, ?, 1, ?, ?)`,
  );
  const select = db.prepare(`SELECT * FROM scenarios WHERE id = ?`);
  const listSql = db.prepare(`SELECT * FROM scenarios ORDER BY created_at DESC, id LIMIT ? OFFSET ?`);
  const countSql = db.prepare(`SELECT COUNT(*) AS c FROM scenarios`);
  const updateSql = db.prepare(
    `UPDATE scenarios SET name = ?, spec = ?, spec_version = spec_version + 1, updated_at = ? WHERE id = ?`,
  );
  const invalidate = db.prepare(
    `UPDATE plans SET valid = 0, invalidated_at = ? WHERE scenario_id = ? AND valid = 1`,
  );

  return {
    create({ name, spec, now }) {
      const ts = (now ?? (() => new Date().toISOString()))();
      const id = randomUUID();
      insert.run(id, name, JSON.stringify(spec), ts, ts);
      return select.get(id) as unknown as ScenarioRow;
    },
    get(id) {
      return select.get(id) as unknown as ScenarioRow | undefined;
    },
    list(limit, offset) {
      const items = listSql.all(limit, offset) as unknown as unknown as ScenarioRow[];
      const total = Number((countSql.get() as { c: number }).c);
      return { items, total, limit, offset };
    },
    update(id, { name, spec, now }) {
      const existing = select.get(id) as unknown as ScenarioRow | undefined;
      if (!existing) return undefined;
      const ts = (now ?? (() => new Date().toISOString()))();
      const mergedSpec = spec !== undefined ? spec : JSON.parse(existing.spec);
      const mergedName = name ?? existing.name;
      const tx = db.prepare(
        `SELECT 1`,
      );
      void tx;
      db.exec("BEGIN IMMEDIATE");
      try {
        updateSql.run(mergedName, JSON.stringify(mergedSpec), ts, id);
        invalidate.run(ts, id);
        db.exec("COMMIT");
      } catch (e) {
        db.exec("ROLLBACK");
        throw e;
      }
      return select.get(id) as unknown as ScenarioRow;
    },
    invalidatePlans(scenarioId, nowIso) {
      invalidate.run(nowIso, scenarioId);
    },
  };
}

export interface PlanRepository {
  create(input: {
    scenarioId: string;
    idempotencyKey?: string;
    params: unknown;
    configStart: unknown;
    configGoal: unknown;
    path: unknown;
    report: unknown;
    now?: () => string;
  }): { row: PlanRow; reused: boolean };
  get(id: string): PlanRow | undefined;
  list(scenarioId: string | undefined, limit: number, offset: number): ListPage<PlanRow>;
}

export function planRepository(db: DatabaseSync): PlanRepository {
  const insert = db.prepare(
    `INSERT INTO plans
      (id, scenario_id, idempotency_key, params, config_start, config_goal, path, report, valid, created_at)
     VALUES (?, ?, ?, ?, ?, ?, ?, ?, 1, ?)`,
  );
  const byKey = db.prepare(`SELECT * FROM plans WHERE idempotency_key = ?`);
  const byId = db.prepare(`SELECT * FROM plans WHERE id = ?`);
  const listAll = db.prepare(
    `SELECT * FROM plans ORDER BY created_at DESC, id LIMIT ? OFFSET ?`,
  );
  const listByScenario = db.prepare(
    `SELECT * FROM plans WHERE scenario_id = ? ORDER BY created_at DESC, id LIMIT ? OFFSET ?`,
  );
  const countAll = db.prepare(`SELECT COUNT(*) AS c FROM plans`);
  const countScenario = db.prepare(`SELECT COUNT(*) AS c FROM plans WHERE scenario_id = ?`);

  return {
    create({ scenarioId, idempotencyKey, params, configStart, configGoal, path, report, now }) {
      const ts = (now ?? (() => new Date().toISOString()))();
      if (idempotencyKey) {
        const existing = byKey.get(idempotencyKey) as unknown as PlanRow | undefined;
        if (existing) return { row: existing, reused: true };
      }
      const id = randomUUID();
      db.exec("BEGIN IMMEDIATE");
      try {
        insert.run(
          id,
          scenarioId,
          idempotencyKey ?? null,
          JSON.stringify(params),
          JSON.stringify(configStart),
          JSON.stringify(configGoal),
          JSON.stringify(path),
          JSON.stringify(report),
          ts,
        );
        db.exec("COMMIT");
      } catch (e) {
        db.exec("ROLLBACK");
        // Unique constraint race => return stored row.
        if (idempotencyKey) {
          const existing = byKey.get(idempotencyKey) as unknown as PlanRow | undefined;
          if (existing) return { row: existing, reused: true };
        }
        throw e;
      }
      return { row: byId.get(id) as unknown as PlanRow, reused: false };
    },
    get(id) {
      return byId.get(id) as unknown as PlanRow | undefined;
    },
    list(scenarioId, limit, offset) {
      const items = scenarioId
        ? (listByScenario.all(scenarioId, limit, offset) as unknown as PlanRow[])
        : (listAll.all(limit, offset) as unknown as PlanRow[]);
      const total = Number(
        ((scenarioId ? countScenario.get(scenarioId) : countAll.get()) as { c: number }).c,
      );
      return { items, total, limit, offset };
    },
  };
}

