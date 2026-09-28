import { DatabaseSync } from "node:sqlite";
import { mkdirSync } from "node:fs";
import { dirname } from "node:path";

export function openDatabase(path: string): DatabaseSync {
  if (path !== ":memory:") {
    mkdirSync(dirname(path), { recursive: true });
  }
  const db = new DatabaseSync(path);
  db.exec("PRAGMA foreign_keys = ON;");
  migrate(db);
  return db;
}

/** Idempotent migration; safe to run on every startup. */
export function migrate(db: DatabaseSync): void {
  db.exec(`
    CREATE TABLE IF NOT EXISTS scenarios (
      id TEXT PRIMARY KEY,
      name TEXT NOT NULL,
      description TEXT NOT NULL DEFAULT '',
      arm_json TEXT NOT NULL,
      obstacles_json TEXT NOT NULL,
      start_json TEXT NOT NULL,
      goal_json TEXT NOT NULL,
      goal_pose_json TEXT NOT NULL,
      scenario_hash TEXT NOT NULL,
      created_at TEXT NOT NULL,
      updated_at TEXT NOT NULL
    );

    CREATE TABLE IF NOT EXISTS params_versions (
      version TEXT PRIMARY KEY,
      params_json TEXT NOT NULL,
      created_at TEXT NOT NULL
    );

    CREATE TABLE IF NOT EXISTS plans (
      id TEXT PRIMARY KEY,
      client_request_id TEXT UNIQUE,
      scenario_id TEXT NOT NULL REFERENCES scenarios(id),
      params_version TEXT NOT NULL,
      scenario_hash TEXT NOT NULL,
      status TEXT NOT NULL,
      seed INTEGER NOT NULL,
      iterations_used INTEGER NOT NULL,
      budget INTEGER NOT NULL,
      path_json TEXT NOT NULL,
      trajectory_json TEXT NOT NULL,
      report_json TEXT NOT NULL,
      input_snapshot_json TEXT NOT NULL,
      valid INTEGER NOT NULL,
      invalid_reason TEXT,
      created_at TEXT NOT NULL
    );

    CREATE INDEX IF NOT EXISTS idx_plans_scenario ON plans(scenario_id);
    CREATE INDEX IF NOT EXISTS idx_scenarios_name ON scenarios(name);
  `);
}
