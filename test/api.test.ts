import { test, before, after } from "node:test";
import assert from "node:assert/strict";
import { rmSync } from "node:fs";
import { join } from "node:path";
import { startApi, type StartedServer } from "../src/api/server.js";
import { generateScenario } from "../src/core/generator.js";

let server: StartedServer;
let base: string;

before(async () => {
  const dir = join(process.cwd(), "tmp", "test-api");
  rmSync(dir, { recursive: true, force: true });
  server = await startApi(join(dir, "api.db"), 0, "127.0.0.1");
  base = `http://127.0.0.1:${server.port}`;
});

after(async () => {
  await server.close();
});

async function api(path: string, init?: RequestInit) {
  const res = await fetch(base + path, {
    ...init,
    headers: { "content-type": "application/json", ...(init?.headers ?? {}) },
  });
  const text = await res.text();
  const body = text ? JSON.parse(text) : null;
  return { status: res.status, body };
}

test("health and preset list", async () => {
  const h = await api("/health");
  assert.equal(h.status, 200);
  assert.equal(h.body.ok, true);
  const p = await api("/presets");
  assert.ok(p.body.presets.includes("narrow"));
});

test("scenario validation returns 400 with field error codes", async () => {
  const bad = await api("/scenarios", { method: "POST", body: JSON.stringify({ name: "" }) });
  assert.equal(bad.status, 400);
  assert.equal(bad.body.error.code, "VALIDATION_ERROR");
  assert.ok(Array.isArray(bad.body.error.details));
});

test("full e2e: create -> plan -> verify -> trajectory -> idempotent replan -> invalidation", async () => {
  const created = await api("/presets/generate", {
    method: "POST",
    body: JSON.stringify({ name: "detour" }),
  });
  assert.equal(created.status, 201);
  const scenarioId = created.body.id;

  const ik = await api(`/scenarios/${scenarioId}/ik?x=1.2&y=0.5`);
  assert.equal(ik.status, 200);
  assert.ok(ik.body.solutions.length >= 1);

  const unreachable = await api(`/scenarios/${scenarioId}/ik?x=50&y=50`);
  assert.equal(unreachable.status, 422);
  assert.equal(unreachable.body.error.code, "UNREACHABLE_TOO_FAR");

  const params = {
    version: "v1",
    step: 0.25,
    resolution: 0.02,
    maxIterations: 4000,
    goalBias: 0.12,
    seed: 7,
  };
  const plan1 = await api(`/scenarios/${scenarioId}/plans`, {
    method: "POST",
    headers: { "idempotency-key": "e2e-key-1" },
    body: JSON.stringify({ params }),
  });
  assert.equal(plan1.status, 201);
  assert.equal(plan1.body.plan.report.status, "success");
  const planId = plan1.body.plan.id;

  const plan2 = await api(`/scenarios/${scenarioId}/plans`, {
    method: "POST",
    headers: { "idempotency-key": "e2e-key-1" },
    body: JSON.stringify({ params }),
  });
  assert.equal(plan2.status, 200);
  assert.equal(plan2.body.reused, true);
  assert.equal(plan2.body.plan.id, planId);

  const verify = await api(`/plans/${planId}/verify`);
  assert.equal(verify.status, 200);
  assert.equal(verify.body.validation.ok, true);
  assert.equal(verify.body.storedValid, true);

  const traj = await api(`/plans/${planId}/trajectory`);
  assert.ok(Array.isArray(traj.body.rows));
  assert.ok(traj.body.rows.length >= 2);
  const csv = await fetch(`${base}/plans/${planId}/trajectory?format=csv`);
  const csvText = await csv.text();
  assert.ok(csvText.startsWith("index,theta1_rad"));

  // Pagination.
  const list = await api(`/scenarios/${scenarioId}/plans?limit=1&offset=0`);
  assert.equal(list.body.limit, 1);
  assert.ok(list.body.total >= 1);

  // Change geometry -> old plan invalid.
  const changedSpec = structuredClone(created.body.spec);
  changedSpec.obstacles = [
    { id: "BIG", x: 1.0, y: 0, radius: 0.6 },
    ...changedSpec.obstacles,
  ];
  const updated = await api(`/scenarios/${scenarioId}`, {
    method: "PUT",
    body: JSON.stringify({ spec: changedSpec }),
  });
  assert.equal(updated.status, 200);
  assert.equal(updated.body.plansInvalidated, true);
  const oldPlan = await api(`/plans/${planId}`);
  assert.equal(oldPlan.body.valid, false);
});

test("failed plan (start collision) is persisted with no collision path", async () => {
  const created = await api("/presets/generate", {
    method: "POST",
    body: JSON.stringify({ name: "start_blocked" }),
  });
  const id = created.body.id;
  const r = await api(`/scenarios/${id}/plans`, {
    method: "POST",
    body: JSON.stringify({
      params: { version: "v1", step: 0.25, resolution: 0.02, maxIterations: 100, goalBias: 0.1, seed: 1 },
    }),
  });
  assert.equal(r.status, 201);
  assert.equal(r.body.plan.report.status, "no_path");
  assert.equal(r.body.plan.report.reason, "START_IN_COLLISION");
  assert.deepEqual(r.body.plan.path, []);
});

test("compare endpoint runs two parameter versions", async () => {
  const created = await api("/presets/generate", {
    method: "POST",
    body: JSON.stringify({ name: "detour" }),
  });
  const r = await api("/compare", {
    method: "POST",
    body: JSON.stringify({
      scenarioId: created.body.id,
      paramsA: { version: "v1", step: 0.25, resolution: 0.02, maxIterations: 1000, goalBias: 0.1, seed: 1 },
      paramsB: { version: "v2", step: 0.15, resolution: 0.01, maxIterations: 4000, goalBias: 0.2, seed: 2 },
    }),
  });
  assert.equal(r.status, 200);
  assert.equal(r.body.a.params.version, "v1");
  assert.equal(r.body.b.params.version, "v2");
});
