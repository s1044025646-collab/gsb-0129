import { segmentCircleCollision } from "./geometry.js";
import { forwardKinematics } from "./kinematics.js";
import type {
  ArmConfig,
  CircleObstacle,
  JointAngles,
  Vec2,
} from "./types.js";

export interface CollisionOptions {
  /** Inflation of each link (half thickness) in world units. */
  linkHalfWidth?: number;
  /** Extra conservative clearance around obstacles. */
  clearance?: number;
}

/**
 * Check the whole arm at a single configuration against every obstacle.
 * Both links are tested as capsules, not only the end-effector point.
 */
export function configCollision(
  arm: ArmConfig,
  obstacles: CircleObstacle[],
  q: JointAngles,
  opts: CollisionOptions = {},
): boolean {
  const inflation = (opts.linkHalfWidth ?? 0) + (opts.clearance ?? 0);
  const fk = forwardKinematics(arm, q);
  const base: Vec2 = { ...arm.base };
  for (const obs of obstacles) {
    // Base itself (treated as point) — usually obstacles won't be at base.
    if (segmentCircleCollision(base, fk.joint, obs, inflation)) return true;
    if (segmentCircleCollision(fk.joint, fk.tip, obs, inflation)) return true;
  }
  return false;
}

/** Find the first colliding obstacle id at a config, or null. */
export function firstCollision(
  arm: ArmConfig,
  obstacles: CircleObstacle[],
  q: JointAngles,
  opts: CollisionOptions = {},
): string | null {
  const inflation = (opts.linkHalfWidth ?? 0) + (opts.clearance ?? 0);
  const fk = forwardKinematics(arm, q);
  const base: Vec2 = { ...arm.base };
  for (const obs of obstacles) {
    if (segmentCircleCollision(base, fk.joint, obs, inflation)) return obs.id;
    if (segmentCircleCollision(fk.joint, fk.tip, obs, inflation)) return obs.id;
  }
  return null;
}

export interface EdgeCheckResult {
  safe: boolean;
  /** Number of intermediate samples evaluated including endpoints. */
  samples: number;
  /** First colliding sample parameter t in [0,1], if any. */
  collisionT: number | null;
  collisionObstacle: string | null;
}

/**
 * Validate a straight-line joint-space edge qA -> qB by dense sampling.
 * Resolution is in radians; the number of subdivisions is derived from the
 * larger joint delta so thin obstacles cannot be tunneled through.
 *
 * Adaptive refinement: additionally bisect once around a found collision to
 * report a tighter t (planning rejects the edge either way).
 */
export function checkEdge(
  arm: ArmConfig,
  obstacles: CircleObstacle[],
  qA: JointAngles,
  qB: JointAngles,
  resolution: number,
  opts: CollisionOptions = {},
): EdgeCheckResult {
  const d1 = qB.theta1 - qA.theta1;
  const d2 = qB.theta2 - qA.theta2;
  const maxDelta = Math.max(Math.abs(d1), Math.abs(d2));
  const steps = Math.max(1, Math.ceil(maxDelta / Math.max(resolution, 1e-9)));

  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    const q: JointAngles = {
      theta1: qA.theta1 + d1 * t,
      theta2: qA.theta2 + d2 * t,
    };
    const hit = firstCollision(arm, obstacles, q, opts);
    if (hit !== null) {
      return { safe: false, samples: i + 1, collisionT: t, collisionObstacle: hit };
    }
  }
  return { safe: true, samples: steps + 1, collisionT: null, collisionObstacle: null };
}

/**
 * Verify an entire path: every consecutive node pair must be collision free
 * under dense sampling, and consecutive configurations must be continuous
 * (joint jumps bounded by resolution * tolerance).
 */
export interface PathValidationResult {
  ok: boolean;
  edges: number;
  totalSamples: number;
  firstFailure: {
    edgeIndex: number;
    t: number;
    obstacle: string | null;
  } | null;
}

export function validatePath(
  arm: ArmConfig,
  obstacles: CircleObstacle[],
  path: JointAngles[],
  resolution: number,
  opts: CollisionOptions = {},
): PathValidationResult {
  if (path.length === 0) {
    return { ok: false, edges: 0, totalSamples: 0, firstFailure: null };
  }
  let totalSamples = 0;
  for (let i = 0; i < path.length - 1; i++) {
    const a = path[i]!;
    const b = path[i + 1]!;
    const res = checkEdge(arm, obstacles, a, b, resolution, opts);
    totalSamples += res.samples;
    if (!res.safe) {
      return {
        ok: false,
        edges: i,
        totalSamples,
        firstFailure: {
          edgeIndex: i,
          t: res.collisionT ?? 0,
          obstacle: res.collisionObstacle,
        },
      };
    }
  }
  return { ok: true, edges: Math.max(0, path.length - 1), totalSamples, firstFailure: null };
}
