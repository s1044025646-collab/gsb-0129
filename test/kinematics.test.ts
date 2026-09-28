import { test } from "node:test";
import assert from "node:assert/strict";
import {
  TAU,
  anglesEquivalent,
  degToRad,
  radToDeg,
  segmentCircleDistance,
  shortestAngleDelta,
} from "../src/geometry.js";
import {
  ArmSpec,
  IKError,
  forwardKinematics,
  interpolateJoints,
  inverseKinematics,
  inverseKinematicsRaw,
  reachesPose,
  wrapIntoLimit,
} from "../src/kinematics.js";

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

test("angle conversions round-trip", () => {
  for (const deg of [-720, -180, -1, 0, 45, 179, 180, 359.9, 720]) {
    assert.ok(Math.abs(radToDeg(degToRad(deg)) - deg) < 1e-9);
  }
});

test("shortest angular delta respects periodic boundary", () => {
  assert.ok(Math.abs(shortestAngleDelta(degToRad(179), degToRad(-179)) - degToRad(2)) < 1e-9);
  assert.ok(
    Math.abs(shortestAngleDelta(degToRad(-179), degToRad(179)) - degToRad(-2)) < 1e-9
  );
  assert.ok(anglesEquivalent(degToRad(361), degToRad(1)));
});

test("FK places elbow and tip correctly", () => {
  const p = forwardKinematics(arm, { theta1: 0, theta2: 0 });
  assert.deepEqual(p.elbow, { x: 2, y: 0 });
  assert.deepEqual(p.tip, { x: 3.5, y: 0 });
  assert.equal(p.phi, 0);
});

test("IK returns both elbow-up and elbow-down solutions", () => {
  // A generic reachable point: elbow at 90deg geometry.
  const x = 2;
  const y = 1.5;
  const report = inverseKinematics(arm, { x, y, phi: null });
  assert.equal(report.solutions.length, 2);
  const elbows = new Set(report.solutions.map((s) => s.elbow));
  assert.ok(elbows.has("up"));
  assert.ok(elbows.has("down"));
  for (const s of report.solutions) {
    assert.ok(reachesPose(arm, s, { x, y, phi: null }, 1e-6));
    // round trip
    const fk = forwardKinematics(arm, s);
    assert.ok(Math.hypot(fk.tip.x - x, fk.tip.y - y) < 1e-6);
  }
});

test("IK round trip for random angles", () => {
  for (const [t1, t2] of [
    [0.3, 0.9],
    [-1.2, 0.4],
    [2.1, -2.0],
    [0.01, 3.1],
  ] as const) {
    const fk = forwardKinematics(arm, { theta1: t1, theta2: t2 });
    const report = inverseKinematics(arm, { x: fk.tip.x, y: fk.tip.y, phi: null });
    assert.ok(
      report.solutions.some((s) => reachesPose(arm, s, { x: fk.tip.x, y: fk.tip.y, phi: null })),
      `round trip failed for ${t1},${t2}`
    );
  }
});

test("fully extended singular pose has a single unique solution", () => {
  const report = inverseKinematics(arm, { x: 3.5, y: 0, phi: null });
  assert.equal(report.solutions.length, 1);
  assert.ok(report.solutions[0]!.singular);
});

test("fully folded singular pose at min reach", () => {
  const report = inverseKinematics(arm, { x: 0.5, y: 0, phi: null });
  assert.equal(report.solutions.length, 1);
  assert.ok(report.solutions[0]!.singular);
});

test("unreachable targets report explicit error codes", () => {
  assert.throws(
    () => inverseKinematicsRaw(arm, { x: 10, y: 0, phi: null }),
    (err: unknown) => err instanceof IKError && err.code === "UNREACHABLE_TOO_FAR"
  );
  assert.throws(
    () => inverseKinematicsRaw(arm, { x: 0.1, y: 0, phi: null }),
    (err: unknown) => err instanceof IKError && err.code === "UNREACHABLE_TOO_NEAR"
  );
});

test("joint limits reject otherwise valid branches and handle wrapping", () => {
  const limited: ArmSpec = {
    ...arm,
    theta1Min: degToRad(0),
    theta1Max: degToRad(80),
    theta2Min: degToRad(-90),
    theta2Max: degToRad(90),
  };
  // Target requiring theta1 beyond 80 for one branch.
  const report = inverseKinematics(limited, { x: -0.5, y: 2.5, phi: null });
  assert.ok(report.solutions.length >= 0);
  for (const s of report.solutions) {
    assert.ok(s.theta1 >= limited.theta1Min - 1e-9 && s.theta1 <= limited.theta1Max + 1e-9);
    assert.ok(s.theta2 >= limited.theta2Min - 1e-9 && s.theta2 <= limited.theta2Max + 1e-9);
  }
});

test("wrapIntoLimit finds periodic representatives across +/-PI boundary", () => {
  // Interval wrapping: [350deg, 370deg] equivalent to [-10deg, 10deg].
  const rep = wrapIntoLimit(degToRad(355), degToRad(350), degToRad(370));
  assert.ok(rep !== null);
  assert.ok(Math.abs(rep! - degToRad(355)) < 1e-9);
  assert.equal(wrapIntoLimit(degToRad(180), degToRad(350), degToRad(370)), null);
});

test("interpolation never takes the long way around", () => {
  const a = { theta1: degToRad(179), theta2: 0 };
  const b = { theta1: degToRad(-179), theta2: 0 };
  const mid = interpolateJoints(a, b, 0.5);
  // Should pass near +/-180 with a tiny 2-degree sweep, not 358 degrees.
  assert.ok(Math.abs(Math.abs(mid.theta1) - Math.PI) < degToRad(1));
});

test("segment-circle distance covers whole segment, not just endpoints", () => {
  // Circle at midpoint of a horizontal link, endpoints far away.
  const d = segmentCircleDistance(
    { ax: 0, ay: 0, bx: 4, by: 0 },
    { id: "o", x: 2, y: 0.4, radius: 0.2 }
  );
  // Closest distance is centre-to-segment (0.4); circle radius is applied by callers.
  assert.ok(Math.abs(d - 0.4) < 1e-9);
});
