import { test } from "node:test";
import assert from "node:assert/strict";
import { TAU, shortestAngleDelta } from "../src/geometry.js";
import { ArmSpec } from "../src/kinematics.js";
import { checkMotionCollision, checkPoseCollision } from "../src/collision.js";
import {
  DEFAULT_PARAMS,
  PlannerParams,
  planPath,
} from "../src/planner.js";
import { generateScenario } from "../src/scenario.js";

const arm: ArmSpec = {
  baseX: 0,
  baseY: 0,
  link1: 2,
  link2: 1.5,
  theta1Min: -Math.PI,
  theta1Max: Math.PI,
  theta2Min: -Math.PI,
  theta2Max: Math.PI,
};

test("tip can be safe while a link collides during the sweep", () => {
  const s = generateScenario("tipSafeLinkHits");
  assert.equal(checkPoseCollision(arm, s.start, s.obstacles, 0).collision, false);
  assert.equal(checkPoseCollision(arm, s.goal, s.obstacles, 0).collision, false);
  const sweep = checkMotionCollision(arm, s.start, s.goal, s.obstacles, {
    maxStep: 0.04,
    safetyMargin: 0.01,
  });
  assert.ok(sweep !== null);
  assert.equal(sweep!.link, "link1");
  // And the planner can route around it.
  const routed = planPath(
    { arm, obstacles: s.obstacles, start: s.start, goal: s.goal },
    { ...DEFAULT_PARAMS, seed: 1, maxIterations: 4000 }
  );
  assert.equal(routed.status, "FOUND");
});

test("start collision status", () => {
  const s = generateScenario("startCollision");
  const result = planPath(
    { arm, obstacles: s.obstacles, start: s.start, goal: s.goal },
    { ...DEFAULT_PARAMS }
  );
  assert.equal(result.status, "START_IN_COLLISION");
  assert.equal(result.path.length, 0);
});

test("clear scene yields direct path", () => {
  const s = generateScenario("clear");
  const result = planPath(
    { arm, obstacles: [], start: s.start, goal: s.goal },
    { ...DEFAULT_PARAMS, seed: 1 }
  );
  assert.equal(result.status, "FOUND");
  assert.ok(result.trajectory.length >= 2);
});

test("detour scene finds collision-free path and it verifies", () => {
  const s = generateScenario("detour");
  const params: PlannerParams = { ...DEFAULT_PARAMS, seed: 2024, maxIterations: 4000 };
  const result = planPath(
    { arm, obstacles: s.obstacles, start: s.start, goal: s.goal },
    params
  );
  assert.equal(result.status, "FOUND");
  for (let i = 0; i < result.trajectory.length - 1; i++) {
    const hit = checkMotionCollision(
      arm,
      result.trajectory[i]!,
      result.trajectory[i + 1]!,
      s.obstacles,
      { maxStep: params.maxCartesianStep, safetyMargin: 0 }
    );
    assert.equal(hit, null, `collision at segment ${i}`);
  }
});

test("fixed seed reproduces identical trajectory", () => {
  const s = generateScenario("narrowPassage");
  const p: PlannerParams = { ...DEFAULT_PARAMS, seed: 777, maxIterations: 4000 };
  const r1 = planPath({ arm, obstacles: s.obstacles, start: s.start, goal: s.goal }, p);
  const r2 = planPath({ arm, obstacles: s.obstacles, start: s.start, goal: s.goal }, p);
  assert.deepEqual(r1.trajectory, r2.trajectory);
  assert.equal(r1.iterationsUsed, r2.iterationsUsed);
});

test("exhausted budget reports NOT_FOUND without a collision path", () => {
  const s = generateScenario("narrowPassage");
  const result = planPath(
    { arm, obstacles: s.obstacles, start: s.start, goal: s.goal },
    { ...DEFAULT_PARAMS, seed: 5, maxIterations: 3, step: 0.04 }
  );
  assert.equal(result.status, "NOT_FOUND");
  assert.equal(result.path.length, 0);
  assert.equal(result.trajectory.length, 0);
  assert.equal(result.iterationsUsed, result.budget);
});

test("adaptive subdivision catches a thin obstacle tunnelled at fixed joint step", () => {
  // Sweep theta1 from 0 to PI with link1 horizontal region. A small circle
  // sits at distance 1 from base, exactly on the swept arc; coarse endpoint
  // sampling could miss it, subdivision must not.
  const thin = { id: "thin", x: 1, y: 0.02, radius: 0.03 };
  const hit = checkMotionCollision(
    arm,
    { theta1: 0, theta2: 0 },
    { theta1: Math.PI, theta2: 0 },
    [thin],
    { maxStep: 0.02, safetyMargin: 0 }
  );
  assert.ok(hit !== null);
  assert.equal(hit!.obstacleId, "thin");
});

test("path continuity: consecutive joint deltas are bounded", () => {
  const s = generateScenario("detour");
  const result = planPath(
    { arm, obstacles: s.obstacles, start: s.start, goal: s.goal },
    { ...DEFAULT_PARAMS, seed: 11, maxIterations: 4000 }
  );
  assert.equal(result.status, "FOUND");
  for (let i = 0; i < result.trajectory.length - 1; i++) {
    const a = result.trajectory[i]!;
    const b = result.trajectory[i + 1]!;
    const d = Math.hypot(
      shortestAngleDelta(a.theta1, b.theta1),
      shortestAngleDelta(a.theta2, b.theta2)
    );
    assert.ok(d <= DEFAULT_PARAMS.step * 1.0001);
  }
});

test("collision boundary: clearance flips exactly around radius", () => {
  // Link along x-axis from 0..2; circle centred (1, y).
  const outside = checkPoseCollision(
    arm,
    { theta1: 0, theta2: 0 },
    [{ id: "o", x: 1, y: 0.5, radius: 0.4 }],
    0
  );
  assert.equal(outside.collision, false);
  const touching = checkPoseCollision(
    arm,
    { theta1: 0, theta2: 0 },
    [{ id: "o", x: 1, y: 0.4, radius: 0.4 }],
    0
  );
  assert.equal(touching.collision, true);
});
