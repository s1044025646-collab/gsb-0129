import { Circle, segmentCircleDistance, segmentHitsCircle } from "./geometry.js";
import {
  ArmSpec,
  JointAngles,
  forwardKinematics,
  interpolateJoints,
} from "./kinematics.js";

export interface CollisionResult {
  collision: boolean;
  obstacleId?: string;
  link?: "link1" | "link2" | "base";
  clearance: number;
}

/** Collision check for a single static pose: both whole links vs every circle. */
export function checkPoseCollision(
  arm: ArmSpec,
  q: JointAngles,
  obstacles: Circle[],
  safetyMargin = 0
): CollisionResult {
  const pts = forwardKinematics(arm, q);
  let minClearance = Infinity;

  const checkLink = (
    ax: number,
    ay: number,
    bx: number,
    by: number,
    link: "link1" | "link2"
  ): CollisionResult | null => {
    for (const o of obstacles) {
      const seg = { ax, ay, bx, by };
      const d = segmentCircleDistance(seg, o);
      const clearance = d - o.radius;
      if (clearance < minClearance) minClearance = clearance;
      if (segmentHitsCircle(seg, o, safetyMargin)) {
        return { collision: true, obstacleId: o.id, link, clearance };
      }
    }
    return null;
  };

  for (const o of obstacles) {
    const d = Math.hypot(pts.base.x - o.x, pts.base.y - o.y);
    const clearance = d - o.radius;
    if (clearance < minClearance) minClearance = clearance;
    if (d <= o.radius + safetyMargin) {
      return { collision: true, obstacleId: o.id, link: "base", clearance };
    }
  }

  const hit1 = checkLink(pts.base.x, pts.base.y, pts.elbow.x, pts.elbow.y, "link1");
  if (hit1) return hit1;
  const hit2 = checkLink(pts.elbow.x, pts.elbow.y, pts.tip.x, pts.tip.y, "link2");
  if (hit2) return hit2;

  return { collision: false, clearance: minClearance };
}

export interface SweepCheckOptions {
  /** Maximum cartesian sweep per joint-space substep; triggers adaptive split. */
  maxStep?: number;
  /** Safety inflation around obstacles. */
  safetyMargin?: number;
}

export interface SweepCollision {
  t: number;
  obstacleId: string;
  link: "link1" | "link2" | "base";
  clearance: number;
}

/**
 * Verify a continuous motion between two configurations. The segment is
 * recursively subdivided until each substep moves no point of the arm more
 * than maxStep in world space, so thin obstacles cannot be tunnelled through.
 */
export function checkMotionCollision(
  arm: ArmSpec,
  from: JointAngles,
  to: JointAngles,
  obstacles: Circle[],
  opts: SweepCheckOptions = {}
): SweepCollision | null {
  const maxStep = opts.maxStep ?? 0.05;
  const safetyMargin = opts.safetyMargin ?? 0;
  return recurse(arm, from, to, obstacles, maxStep, safetyMargin, 0, 1, 0);
}

function recurse(
  arm: ArmSpec,
  a: JointAngles,
  b: JointAngles,
  obstacles: Circle[],
  maxStep: number,
  safetyMargin: number,
  t0: number,
  t1: number,
  depth: number
): SweepCollision | null {
  const pa = forwardKinematics(arm, a);
  const pb = forwardKinematics(arm, b);
  const moves = [
    Math.hypot(pb.elbow.x - pa.elbow.x, pb.elbow.y - pa.elbow.y),
    Math.hypot(pb.tip.x - pa.tip.x, pb.tip.y - pa.tip.y),
  ];
  const maxMove = Math.max(...moves);

  const midT = (t0 + t1) / 2;
  // Must match the planner's interpolation (shortest signed angle delta),
  // not arithmetic mean, or periodic-boundary paths mismatch.
  const mid = interpolateJoints(a, b, 0.5);

  if (maxMove > maxStep && depth < 40) {
    const left = recurse(arm, a, mid, obstacles, maxStep, safetyMargin, t0, midT, depth + 1);
    if (left) return left;
    return recurse(arm, mid, b, obstacles, maxStep, safetyMargin, midT, t1, depth + 1);
  }

  // Sample both endpoints at the leaf so no gap exists between adjacent
  // substeps (nonlinear joint->cartesian mapping).
  for (const [q, t] of [
    [a, t0],
    [mid, midT],
    [b, t1],
  ] as const) {
    const hit = checkPoseCollision(arm, q, obstacles, safetyMargin);
    if (hit.collision) {
      return {
        t,
        obstacleId: hit.obstacleId!,
        link: hit.link!,
        clearance: hit.clearance,
      };
    }
  }
  return null;
}
