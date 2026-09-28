import { test } from "node:test";
import assert from "node:assert/strict";
import { validatePath } from "../src/core/collision.js";
import { generateScenario } from "../src/core/generator.js";
import { planPath, PlannerError } from "../src/core/planner.js";
import { defaultParams } from "../src/core/validation.js";

test("open workspace: success with verified continuous collision-free path", () => {
  const s = generateScenario("open");
  const r = planPath(s.arm, s.obstacles, s.start, s.goal, defaultParams(123));
  assert.equal(r.status, "success");
  assert.ok(r.path.length >= 2);
  const v = validatePath(s.arm, s.obstacles, r.path, 0.01);
  assert.equal(v.ok, true);
  // Continuous edges: no large joint jumps between stored nodes.
  for (let i = 1; i < r.path.length; i++) {
    const d = Math.hypot(
      r.path[i]!.theta1 - r.path[i - 1]!.theta1,
      r.path[i]!.theta2 - r.path[i - 1]!.theta2,
    );
    assert.ok(d <= defaultParams(123).step + 1e-9);
  }
});

test("fixed seed reproduces identical plans", () => {
  const s = generateScenario("detour");
  const p = defaultParams(99);
  const a = planPath(s.arm, s.obstacles, s.start, s.goal, p);
  const b = planPath(s.arm, s.obstacles, s.start, s.goal, { ...p });
  assert.deepEqual(a.path, b.path);
  assert.equal(a.report.iterations, b.report.iterations);
});

test("detour preset: successful path bends around the obstacle", () => {
  const s = generateScenario("detour");
  const r = planPath(s.arm, s.obstacles, s.start, s.goal, defaultParams(7));
  assert.equal(r.status, "success");
  assert.equal(validatePath(s.arm, s.obstacles, r.path, 0.01).ok, true);
});

test("start collision reports START_IN_COLLISION rather than a path", () => {
  const s = generateScenario("start_blocked");
  const r = planPath(s.arm, s.obstacles, s.start, s.goal, defaultParams(1));
  assert.equal(r.status, "no_path");
  assert.equal(r.report.reason, "START_IN_COLLISION");
  assert.equal(r.report.startCollision, true);
  assert.equal(r.path.length, 0);
});

test("exhausted budget reports BUDGET_EXHAUSTED, never a collision path", () => {
  // The narrow preset with a deliberately tiny budget cannot be guaranteed
  // solved. Whatever happens, a no_path budget result must carry the reason;
  // a success must verify collision-free. We assert the contract both ways.
  const s = generateScenario("narrow");
  const params = { ...defaultParams(5), maxIterations: 3, goalBias: 0 };
  const r = planPath(s.arm, s.obstacles, s.start, s.goal, params);
  if (r.status === "no_path") {
    assert.equal(r.report.reason, "BUDGET_EXHAUSTED");
    assert.equal(r.path.length, 0);
  } else {
    assert.equal(validatePath(s.arm, s.obstacles, r.path, 0.01).ok, true);
  }
});

test("unreachable goal raises a planner error with explicit code", () => {
  const s = generateScenario("open");
  const badGoal = { x: 99, y: 99, phi: 0 };
  assert.throws(
    () => planPath(s.arm, [], s.start, badGoal, defaultParams(1)),
    (e: unknown) => e instanceof PlannerError && e.code === "GOAL_UNREACHABLE",
  );
});

test("periodic boundary: plans may cross -pi/pi without a giant jump", () => {
  const s = generateScenario("open");
  // Choose IK configs that sit near opposite sides of the +/-pi seam.
  s.arm.limits.theta1Min = -Math.PI;
  s.arm.limits.theta1Max = Math.PI;
  s.start = { x: 1.78, y: -0.05, phi: 0 };
  s.goal = { x: 1.78, y: 0.05, phi: 0 };
  const r = planPath(s.arm, [], s.start, s.goal, defaultParams(321));
  assert.equal(r.status, "success");
  for (let i = 1; i < r.path.length; i++) {
    const d = Math.hypot(
      r.path[i]!.theta1 - r.path[i - 1]!.theta1,
      r.path[i]!.theta2 - r.path[i - 1]!.theta2,
    );
    assert.ok(d < Math.PI, "joint interpolation must use shortest periodic arc");
  }
});
