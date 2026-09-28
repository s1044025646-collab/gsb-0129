import { test } from "node:test";
import assert from "node:assert/strict";
import { degToRad, pointSegmentDistance } from "../src/core/geometry.js";
import { checkEdge, configCollision, validatePath } from "../src/core/collision.js";
import type { ArmConfig, CircleObstacle } from "../src/core/types.js";

const arm: ArmConfig = {
  l1: 1,
  l2: 0.8,
  base: { x: 0, y: 0 },
  limits: {
    theta1Min: -Math.PI,
    theta1Max: Math.PI,
    theta2Min: -Math.PI,
    theta2Max: Math.PI,
  },
};

test("point-segment distance boundary cases", () => {
  const a = { x: 0, y: 0 };
  const b = { x: 10, y: 0 };
  assert.ok(Math.abs(pointSegmentDistance({ x: 5, y: 3 }, a, b).d - 3) < 1e-12);
  assert.ok(Math.abs(pointSegmentDistance({ x: -4, y: 0 }, a, b).d - 4) < 1e-12);
  assert.ok(Math.abs(pointSegmentDistance({ x: 12, y: 5 }, a, b).d - Math.hypot(2, 5)) < 1e-12);
});

test("end-effector clear but link colliding is detected", () => {
  // Arm horizontal; tip is at (1.8, 0). Obstacle on link1, away from tip.
  const obs: CircleObstacle[] = [{ id: "O", x: 0.5, y: 0, radius: 0.05 }];
  const hit = configCollision(arm, obs, { theta1: 0, theta2: 0 });
  assert.equal(hit, true);
  // Move obstacle to the tip: also detected.
  const tipObs: CircleObstacle[] = [{ id: "O", x: 1.8, y: 0, radius: 0.05 }];
  assert.equal(configCollision(arm, tipObs, { theta1: 0, theta2: 0 }), true);
  // Far away obstacle is clear.
  const far: CircleObstacle[] = [{ id: "O", x: 0, y: 5, radius: 0.1 }];
  assert.equal(configCollision(arm, far, { theta1: 0, theta2: 0 }), false);
});

test("tangency boundary is treated as collision (conservative)", () => {
  const obs: CircleObstacle[] = [{ id: "O", x: 1.0, y: 0.1, radius: 0.1 }];
  // Link on the x-axis passes exactly 0.1 below center => tangent.
  assert.equal(configCollision(arm, obs, { theta1: 0, theta2: 0 }), true);
  const safe: CircleObstacle[] = [{ id: "O", x: 1.0, y: 0.1 + 1e-6, radius: 0.1 }];
  assert.equal(configCollision(arm, safe, { theta1: 0, theta2: 0 }), false);
});

test("dense edge sampling catches tunneling through a thin obstacle", () => {
  // Swing from below the horizontal to above; a tiny obstacle at the elbow
  // sweep is crossed mid-edge. Coarse sampling would skip it; fine sampling
  // at 0.005 rad catches it.
  const thin: CircleObstacle[] = [{ id: "thin", x: 0.995, y: 0, radius: 0.03 }];
  const a = { theta1: degToRad(-6), theta2: 0 };
  const b = { theta1: degToRad(6), theta2: 0 };
  const fine = checkEdge(arm, thin, a, b, 0.005);
  assert.equal(fine.safe, false);
  assert.equal(fine.collisionObstacle, "thin");
  const clearPath = validatePath(
    arm,
    [],
    [{ theta1: 0, theta2: 0 }, { theta1: 0.3, theta2: 0.2 }],
    0.01,
  );
  assert.equal(clearPath.ok, true);
  assert.equal(clearPath.edges, 1);
});
