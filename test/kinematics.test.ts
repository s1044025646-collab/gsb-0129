import { test } from "node:test";
import assert from "node:assert/strict";
import { degToRad, radToDeg, wrapAngle, nearestEquivalent } from "../src/core/geometry.js";
import {
  forwardKinematics,
  inverseKinematics,
  IkError,
  filterByLimits,
  solvePose,
} from "../src/core/kinematics.js";
import type { ArmConfig } from "../src/core/types.js";

const arm: ArmConfig = {
  l1: 1,
  l2: 0.8,
  base: { x: 0, y: 0 },
  limits: {
    theta1Min: degToRad(-170),
    theta1Max: degToRad(170),
    theta2Min: degToRad(-160),
    theta2Max: degToRad(160),
  },
};

test("angle conversion helpers round-trip", () => {
  for (const d of [-359, -180, -90, 0, 45, 179, 360, 720]) {
    const back = radToDeg(degToRad(d));
    assert.ok(Math.abs(((back - d + 540) % 360) - 180) < 1e-9 || Math.abs(back - d) < 1e-9);
  }
  assert.ok(Math.abs(wrapAngle(degToRad(370)) - degToRad(10)) < 1e-12);
  assert.ok(Math.abs(wrapAngle(degToRad(-370)) - degToRad(-10)) < 1e-12);
});

test("IK gives two elbow solutions and FK round-trips", () => {
  const target = { x: 1.2, y: 0.5 };
  const sols = inverseKinematics(arm, target);
  assert.equal(sols.length, 2);
  assert.deepEqual(sols.map((s) => s.elbow).sort(), ["down", "up"]);
  for (const s of sols) {
    const fk = forwardKinematics(arm, s);
    assert.ok(Math.abs(fk.tip.x - target.x) < 1e-9);
    assert.ok(Math.abs(fk.tip.y - target.y) < 1e-9);
  }
});

test("fully stretched arm is singular: single coincident solution", () => {
  const target = { x: 1.8, y: 0 };
  const sols = inverseKinematics(arm, target);
  assert.equal(sols.length, 1);
  assert.ok(Math.abs(sols[0]!.theta2) < 1e-9);
  const fk = forwardKinematics(arm, sols[0]!);
  assert.ok(Math.abs(fk.tip.x - 1.8) < 1e-9);
});

test("fully folded arm at inner reach is singular", () => {
  const target = { x: 0.2, y: 0 };
  const sols = inverseKinematics(arm, target);
  assert.equal(sols.length, 1);
  assert.ok(Math.abs(Math.abs(sols[0]!.theta2) - Math.PI) < 1e-9);
});

test("unreachable targets raise explicit IK errors", () => {
  assert.throws(() => inverseKinematics(arm, { x: 5, y: 5 }), (e: unknown) => e instanceof IkError && e.code === "UNREACHABLE_TOO_FAR");
  assert.throws(() => inverseKinematics(arm, { x: 0.05, y: 0 }), (e: unknown) => e instanceof IkError && e.code === "UNREACHABLE_TOO_NEAR");
});

test("joint limits filter elbow branches and periodic wrapping is honored", () => {
  const target = { x: 1.2, y: 0.5 };
  const sols = inverseKinematics(arm, target);
  const tight: ArmConfig = {
    ...arm,
    limits: { theta1Min: 0, theta1Max: degToRad(170), theta2Min: degToRad(-160), theta2Max: 0 },
  };
  const allowed = filterByLimits(sols, tight.limits);
  const elbows = allowed.map((s) => s.elbow);
  assert.ok(elbows.includes("down"));
  assert.ok(!elbows.includes("up"));
  assert.ok(allowed.every((s) => s.theta1 >= 0 && s.theta2 <= 0));
  // Periodic boundary: an angle just beyond +pi equivalent to just below -pi.
  const eq = nearestEquivalent(degToRad(181), degToRad(-179));
  assert.ok(Math.abs(eq - degToRad(-179)) < 1e-9);
});

test("solvePose reports end-effector orientation error", () => {
  const sols = solvePose(arm, { x: 1.2, y: 0.5, phi: 0.1 });
  assert.equal(sols.length, 2);
  for (const s of sols) assert.equal(typeof s.phiError, "number");
});
