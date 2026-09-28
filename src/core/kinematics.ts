import { TAU, nearestEquivalent } from "./geometry.js";
import type {
  ArmConfig,
  JointAngles,
  JointLimits,
  Pose,
  Vec2,
} from "./types.js";

const EPS = 1e-9;

export interface FkResult {
  joint: Vec2; // elbow position
  tip: Vec2; // end-effector position
  phi: number; // end-effector orientation
}

/** Forward kinematics for a 2R planar arm. */
export function forwardKinematics(arm: ArmConfig, q: JointAngles): FkResult {
  const { base, l1, l2 } = arm;
  const c1 = Math.cos(q.theta1);
  const s1 = Math.sin(q.theta1);
  const c12 = Math.cos(q.theta1 + q.theta2);
  const s12 = Math.sin(q.theta1 + q.theta2);
  const joint: Vec2 = { x: base.x + l1 * c1, y: base.y + l1 * s1 };
  const tip: Vec2 = {
    x: base.x + l1 * c1 + l2 * c12,
    y: base.y + l1 * s1 + l2 * s12,
  };
  return { joint, tip, phi: q.theta1 + q.theta2 };
}

export type IkErrorCode = "UNREACHABLE_TOO_FAR" | "UNREACHABLE_TOO_NEAR" | "INVALID_ARM";

export class IkError extends Error {
  code: IkErrorCode;
  constructor(code: IkErrorCode, message: string) {
    super(message);
    this.name = "IkError";
    this.code = code;
  }
}

export interface IkSolution extends JointAngles {
  /** "up" = elbow above the base->tip line (positive offset), "down" otherwise. */
  elbow: "up" | "down";
}

/**
 * Inverse kinematics ignoring the end-effector orientation: a 2R arm only
 * constrains the tip position; phi is determined by theta1+theta2.
 *
 * Enumerates the two elbow configurations (elbow-up / elbow-down).
 * Fully-stretched and fully-folded postures are singular: the two solutions
 * coincide and are returned as a single solution tagged "up".
 */
export function inverseKinematics(arm: ArmConfig, target: Vec2): IkSolution[] {
  const { base, l1, l2 } = arm;
  if (!(l1 > 0) || !(l2 > 0)) {
    throw new IkError("INVALID_ARM", "Link lengths must be positive");
  }
  const dx = target.x - base.x;
  const dy = target.y - base.y;
  const r = Math.hypot(dx, dy);
  const reach = l1 + l2;
  const fold = Math.abs(l1 - l2);

  if (r > reach + 1e-9) {
    throw new IkError(
      "UNREACHABLE_TOO_FAR",
      `Target radius ${r.toFixed(4)} exceeds reach ${reach.toFixed(4)}`,
    );
  }
  if (r < fold - 1e-9) {
    throw new IkError(
      "UNREACHABLE_TOO_NEAR",
      `Target radius ${r.toFixed(4)} inside inner workspace ${fold.toFixed(4)}`,
    );
  }

  const baseAngle = Math.atan2(dy, dx);

  // Singular configurations: fully stretched or fully folded.
  let cosTheta2: number;
  if (Math.abs(r - reach) <= 1e-9) {
    cosTheta2 = 1;
  } else if (Math.abs(r - fold) <= 1e-9) {
    cosTheta2 = l1 >= l2 ? -1 : -1;
  } else {
    cosTheta2 = (r * r - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  }
  cosTheta2 = Math.max(-1, Math.min(1, cosTheta2));
  const sinTheta2 = Math.sqrt(Math.max(0, 1 - cosTheta2 * cosTheta2));
  const theta2Up = Math.atan2(sinTheta2, cosTheta2);
  const theta2Down = Math.atan2(-sinTheta2, cosTheta2);

  const singular = sinTheta2 <= EPS;

  const make = (theta2: number, elbow: "up" | "down"): IkSolution => {
    // Angle of link1 from law of cosines:
    // phi1 = atan2(l2 sin t2, l1 + l2 cos t2)
    const phi1 = Math.atan2(l2 * Math.sin(theta2), l1 + l2 * Math.cos(theta2));
    const theta1 = baseAngle - phi1;
    return { theta1, theta2, elbow };
  };

  if (singular) {
    // Both elbows coincide; return one canonical solution.
    return [make(theta2Up, "up")];
  }
  return [make(theta2Up, "up"), make(theta2Down, "down")];
}

/** True when angle a (mod 2pi) lies within the given limit interval.
 * The interval is interpreted as a normal real interval; periodic equivalents
 * of `a` are tested against it. */
export function angleWithinLimit(a: number, min: number, max: number): boolean {
  if (max < min) return false;
  // Find the equivalent of a nearest to the interval midpoint and test.
  const mid = (min + max) / 2;
  const equiv = nearestEquivalent(a, mid);
  return equiv >= min - 1e-9 && equiv <= max + 1e-9;
}

/**
 * Move an angle by multiples of 2*pi so that it lies within [min, max].
 * Returns null if no periodic equivalent lies in range.
 */
export function wrapIntoLimit(a: number, min: number, max: number): number | null {
  if (!angleWithinLimit(a, min, max)) return null;
  const mid = (min + max) / 2;
  return nearestEquivalent(a, mid);
}

/** Filter IK solutions by joint limits, choosing periodic equivalents. */
export function filterByLimits(
  solutions: IkSolution[],
  limits: JointLimits,
): (IkSolution & { inLimits: true })[] {
  const out: (IkSolution & { inLimits: true })[] = [];
  for (const s of solutions) {
    const t1 = wrapIntoLimit(s.theta1, limits.theta1Min, limits.theta1Max);
    const t2 = wrapIntoLimit(s.theta2, limits.theta2Min, limits.theta2Max);
    if (t1 === null || t2 === null) continue;
    out.push({ theta1: t1, theta2: t2, elbow: s.elbow, inLimits: true as const });
  }
  return out;
}

export function inLimits(q: JointAngles, limits: JointLimits): boolean {
  return angleWithinLimit(q.theta1, limits.theta1Min, limits.theta1Max) &&
    angleWithinLimit(q.theta2, limits.theta2Min, limits.theta2Max);
}

/**
 * Choose tip IK configurations for start/goal poses. The end-effector
 * orientation is reported but, for a 2R arm, cannot be independently chosen;
 * callers may compare it with pose.phi.
 */
export function solvePose(
  arm: ArmConfig,
  pose: Pose,
): (IkSolution & { phiError: number })[] {
  const sols = inverseKinematics(arm, { x: pose.x, y: pose.y });
  return sols.map((s) => {
    const phi = s.theta1 + s.theta2;
    // Signed wrapped angle difference.
    let d = pose.phi - phi;
    d = Math.atan2(Math.sin(d), Math.cos(d));
    return { ...s, phiError: d };
  });
}

export { TAU };
