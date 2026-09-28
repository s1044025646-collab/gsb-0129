import { test } from "node:test";
import assert from "node:assert/strict";
import { rmSync } from "node:fs";
import { join } from "node:path";
import { Server } from "node:http";
import { startServer } from "../src/server.js";
import { generateScenario } from "../src/scenario.js";
import { TAU } from "../src/geometry.js";

const dbPath = join("data", "test-api.sqlite");

let server: Server;
let base: string;

test.before(async () => {
  rmSync(dbPath, { force: true });
  const started = await startServer(dbPath, 0);
  server = started.server;
  base = `http://127.0.0.1:${started.port}`;
});

test.after(() => {
  server.close();
});

async function api(method: string, path: string, body?: unknown) {
  const res = await fetch(`${base}${path}`, {
    method,
    headers: { "content-type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  const json = (await res.json()) as Record<string, unknown>;
  return { status: res.status, json };
}

test("health and 404 error code", async () => {
  const health = await api("GET", "/health");
  assert.equal(health.status, 200);
  const missing = await api("GET", "/api/plans/nope");
  assert.equal(missing.status, 404);
  assert.equal((missing.json.error as { code: string }).code, "NOT_FOUND");
});

test("validation error has explicit code and field details", async () => {
  const res = await api("POST", "/api/fk", {
    arm: { baseX: 0, baseY: 0, link1: -1, link2: 1 },
    joints: { theta1: 0, theta2: 0 },
  });
  assert.equal(res.status, 400);
  assert.equal((res.json.error as { code: string }).code, "VALIDATION_ERROR");
});

test("IK unreachable returns 422 with error code", async () => {
  const arm = {
    baseX: 0,
    baseY: 0,
    link1: 2,
    link2: 1.5,
    theta1Min: -Math.PI,
    theta1Max: Math.PI,
    theta2Min: -Math.PI,
    theta2Max: Math.PI,
  };
  const res = await api("POST", "/api/ik", { arm, pose: { x: 99, y: 99, phi: null } });
  assert.equal(res.status, 422);
  assert.equal((res.json.error as { code: string }).code, "UNREACHABLE_TOO_FAR");
});

test("scenario create, plan, verify, export end to end", async () => {
  const s = generateScenario("clear");
  const created = await api("POST", "/api/scenarios", {
    id: "api-clear",
    name: s.name,
    description: s.description,
    arm: s.arm,
    obstacles: s.obstacles,
    start: s.start,
    goal: s.goal,
    goalPose: s.goalPose,
  });
  assert.equal(created.status, 201);

  const plan1 = await api("POST", "/api/plans", {
    scenarioId: "api-clear",
    clientRequestId: "e2e-1",
    params: { seed: 123 },
  });
  assert.equal(plan1.status, 201);
  const planId = (plan1.json.plan as { id: string }).id;

  const plan2 = await api("POST", "/api/plans", {
    scenarioId: "api-clear",
    clientRequestId: "e2e-1",
    params: { seed: 999 },
  });
  assert.equal(plan2.status, 200);
  assert.equal(plan2.json.reused, true);

  const verify = await api("POST", `/api/plans/${planId}/verify`);
  assert.equal(verify.status, 200);
  assert.equal((verify.json.verification as { ok: boolean }).ok, true);

  const exported = await api("GET", `/api/plans/${planId}/export`);
  assert.equal(exported.status, 200);
  assert.ok(
    Array.isArray((exported.json.joints as unknown[])) &&
      Array.isArray(exported.json.cartesian as unknown[])
  );
});

test("pagination query params enforced", async () => {
  const res = await api("GET", "/api/scenarios?limit=999");
  assert.equal(res.status, 400);
});

test("compare endpoint", async () => {
  const res = await api("POST", "/api/compare", {
    scenarioId: "api-clear",
    paramsA: { seed: 1, paramsVersion: "a" },
    paramsB: { seed: 2, paramsVersion: "b" },
  });
  assert.equal(res.status, 200);
  assert.ok("sameStatus" in res.json);
});
