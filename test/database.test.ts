import { test } from "node:test";
import assert from "node:assert/strict";
import { rmSync, existsSync } from "node:fs";
import { join } from "node:path";
import {
  migrate,
  openDatabase,
  scenarioRepository,
  planRepository,
} from "../src/db/database.js";
import { generateScenario } from "../src/core/generator.js";

function tmpDb(name: string): string {
  const dir = join(process.cwd(), "tmp", "test-db");
  rmSync(join(dir, name), { force: true });
  for (const suffix of ["-wal", "-shm"]) {
    rmSync(join(dir, name + suffix), { force: true });
  }
  return join(dir, name);
}

test("migrations are repeatable and data survives reopen", () => {
  const file = tmpDb("persist.db");
  const db1 = openDatabase(file);
  const scenarios1 = scenarioRepository(db1);
  const plans1 = planRepository(db1);
  const spec = generateScenario("open");
  const row = scenarios1.create({ name: "p1", spec });
  const { row: plan, reused } = plans1.create({
    scenarioId: row.id,
    params: { v: 1 },
    configStart: { theta1: 0, theta2: 0 },
    configGoal: { theta1: 1, theta2: 1 },
    path: [{ theta1: 0, theta2: 0 }, { theta1: 1, theta2: 1 }],
    report: { status: "success" },
  });
  assert.equal(reused, false);
  db1.close();

  // Reopen: migrations run again (idempotent), rows persist.
  const db2 = openDatabase(file);
  migrate(db2);
  const scenarios2 = scenarioRepository(db2);
  const plans2 = planRepository(db2);
  assert.ok(scenarios2.get(row.id));
  assert.ok(plans2.get(plan.id));
  db2.close();
  assert.ok(existsSync(file));
});

test("idempotency key returns the same stored plan on repeat submit", () => {
  const file = tmpDb("idem.db");
  const db = openDatabase(file);
  const scenarios = scenarioRepository(db);
  const plans = planRepository(db);
  const sc = scenarios.create({ name: "i", spec: generateScenario("open") });
  const payload = {
    scenarioId: sc.id,
    idempotencyKey: "key-xyz",
    params: { seed: 1 },
    configStart: { theta1: 0, theta2: 0 },
    configGoal: { theta1: 1, theta2: 1 },
    path: [{ theta1: 0, theta2: 0 }],
    report: { status: "success" },
  };
  const first = plans.create(payload);
  const second = plans.create(payload);
  const third = plans.create({ ...payload, params: { seed: 2 } });
  assert.equal(first.reused, false);
  assert.equal(second.reused, true);
  assert.equal(third.reused, true);
  assert.equal(first.row.id, second.row.id);
  assert.equal(first.row.id, third.row.id);
  // Snapshot from first submit is preserved.
  assert.equal(JSON.parse(third.row.params).seed, 1);
  db.close();
});

test("scenario update invalidates old plans in the same transaction", () => {
  const file = tmpDb("inval.db");
  const db = openDatabase(file);
  const scenarios = scenarioRepository(db);
  const plans = planRepository(db);
  const sc = scenarios.create({ name: "v", spec: generateScenario("open") });
  const p = plans.create({
    scenarioId: sc.id,
    params: {},
    configStart: {},
    configGoal: {},
    path: [],
    report: { status: "success" },
  }).row;
  assert.equal(p.valid, 1);

  const changed = structuredClone(generateScenario("detour"));
  const updated = scenarios.update(sc.id, { spec: changed })!;
  assert.equal(updated.spec_version, 2);
  const reloaded = plans.get(p.id)!;
  assert.equal(reloaded.valid, 0);
  assert.ok(reloaded.invalidated_at);
  // Stored path snapshot is untouched even though geometry changed.
  assert.equal(JSON.parse(reloaded.path).length, 0);
  db.close();
});

test("transaction rollback on failure leaves prior state intact", () => {
  const file = tmpDb("rollback.db");
  const db = openDatabase(file);
  const scenarios = scenarioRepository(db);
  const sc = scenarios.create({ name: "rb", spec: generateScenario("open") });
  const beforeVersion = scenarios.get(sc.id)!.spec_version;

  // Force a failure inside the update transaction by breaking SQL with a bad id type.
  assert.throws(() => {
    db.exec("BEGIN IMMEDIATE");
    db.prepare("UPDATE scenarios SET spec_version = spec_version + 1 WHERE id = ?").run(sc.id);
    db.prepare("INSERT INTO plans (id) VALUES (?)").run(null); // NOT NULL / missing columns => error
    db.exec("COMMIT");
  });
  db.exec("ROLLBACK");
  const after = scenarios.get(sc.id)!;
  assert.equal(after.spec_version, beforeVersion, "partial update must have rolled back");
  db.close();
});

test("pagination respects limit/offset and reports total", () => {
  const file = tmpDb("page.db");
  const db = openDatabase(file);
  const scenarios = scenarioRepository(db);
  for (let i = 0; i < 5; i++) scenarios.create({ name: `s${i}`, spec: generateScenario("open") });
  const page1 = scenarios.list(2, 0);
  const page2 = scenarios.list(2, 2);
  assert.equal(page1.items.length, 2);
  assert.equal(page2.items.length, 2);
  assert.equal(page1.total, 5);
  assert.notEqual(page1.items[0]!.id, page2.items[0]!.id);
  db.close();
});
