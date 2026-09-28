import { test } from "node:test";
import assert from "node:assert/strict";
import { rmSync } from "node:fs";
import { join } from "node:path";
import { openDatabase } from "../src/db.js";
import { Store } from "../src/store.js";
import { KinematicsService } from "../src/service.js";
import { generateScenario } from "../src/scenario.js";
import { DEFAULT_PARAMS } from "../src/planner.js";

function freshStore(path: string, now = "2026-01-01T00:00:00Z") {
  rmSync(path, { force: true });
  const db = openDatabase(path);
  const store = new Store(db, () => new Date(now));
  return { db, store };
}

test("transaction rollback leaves no trace", () => {
  const path = join("data", "test-rollback.sqlite");
  const { db, store } = freshStore(path);
  const s = generateScenario("clear");
  assert.throws(() =>
    store.transaction(() => {
      store.upsertScenario({
        id: "tx",
        name: s.name,
        description: s.description,
        arm: s.arm,
        obstacles: s.obstacles,
        start: s.start,
        goal: s.goal,
        goalPose: s.goalPose,
      });
      throw new Error("boom");
    }, false)
  );
  assert.equal(store.getScenario("tx"), null);
  db.close();
});

test("explicit rollback flag also discards work", () => {
  const path = join("data", "test-rollback2.sqlite");
  const { db, store } = freshStore(path);
  const s = generateScenario("clear");
  store.transaction(
    () =>
      store.upsertScenario({
        id: "tx2",
        name: s.name,
        description: "",
        arm: s.arm,
        obstacles: s.obstacles,
        start: s.start,
        goal: s.goal,
        goalPose: s.goalPose,
      }),
    true
  );
  assert.equal(store.getScenario("tx2"), null);
  db.close();
});

test("state persists after reopening the database", () => {
  const path = join("data", "test-persist.sqlite");
  const s = generateScenario("clear");
  {
    const { db, store } = freshStore(path);
    store.upsertScenario({
      id: "persist",
      name: s.name,
      description: "",
      arm: s.arm,
      obstacles: s.obstacles,
      start: s.start,
      goal: s.goal,
      goalPose: s.goalPose,
    });
    db.close();
  }
  const db2 = openDatabase(path);
  const store2 = new Store(db2);
  const reloaded = store2.getScenario("persist");
  assert.ok(reloaded);
  assert.deepEqual(reloaded!.arm, s.arm);
  db2.close();
});

test("idempotent plan submission with client request id", () => {
  const path = join("data", "test-idem.sqlite");
  const { db, store } = freshStore(path);
  const service = new KinematicsService(store);
  const s = store.upsertScenario({
    id: "idem",
    ...scenarioPayload("clear"),
  });
  const r1 = service.plan(s, { ...DEFAULT_PARAMS, seed: 3 }, "key-123");
  const r2 = service.plan(s, { ...DEFAULT_PARAMS, seed: 999 }, "key-123");
  assert.equal(r1.reused, false);
  assert.equal(r2.reused, true);
  assert.equal(r1.record.id, r2.record.id);
  assert.equal(r2.record.seed, r1.record.seed);
  db.close();
});

test("changing geometry invalidates old plans but keeps snapshot", () => {
  const path = join("data", "test-invalid.sqlite");
  const { db, store } = freshStore(path);
  const service = new KinematicsService(store);
  let s = store.upsertScenario({ id: "inv", ...scenarioPayload("clear") });
  const plan = service.plan(s, { ...DEFAULT_PARAMS, seed: 4 }, "k-inv");
  assert.equal(plan.record.valid, true);

  const changed = generateScenario("clear");
  changed.obstacles = [{ id: "big", x: 0, y: 1, radius: 0.6 }];
  s = store.upsertScenario({ id: "inv", ...scenarioPayloadFrom(changed) });

  const refreshed = store.refreshPlanValidity(plan.record.id)!;
  assert.equal(refreshed.valid, false);
  assert.equal(refreshed.invalidReason, "SCENARIO_CHANGED");
  // Snapshot preserves the original planning inputs.
  assert.deepEqual(refreshed.inputSnapshot.obstacles, []);
  db.close();
});

test("pagination limits and offsets", () => {
  const path = join("data", "test-page.sqlite");
  const { db, store } = freshStore(path);
  for (let i = 0; i < 5; i++) {
    store.upsertScenario({ id: `p${i}`, ...scenarioPayload("clear"), name: `p${i}` });
  }
  const page = store.listScenarios(2, 1);
  assert.equal(page.total, 5);
  assert.equal(page.items.length, 2);
  assert.equal(page.limit, 2);
  assert.equal(page.offset, 1);
  db.close();
});

function scenarioPayload(kind: Parameters<typeof generateScenario>[0]) {
  const s = generateScenario(kind);
  return {
    name: s.name,
    description: s.description,
    arm: s.arm,
    obstacles: s.obstacles,
    start: s.start,
    goal: s.goal,
    goalPose: s.goalPose,
  };
}

function scenarioPayloadFrom(s: ReturnType<typeof generateScenario>) {
  return {
    name: s.name,
    description: s.description,
    arm: s.arm,
    obstacles: s.obstacles,
    start: s.start,
    goal: s.goal,
    goalPose: s.goalPose,
  };
}
