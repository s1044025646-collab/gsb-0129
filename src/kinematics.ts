import {
  TAU,
  anglesEquivalent,
  distance,
  normalizeAngle0To2Pi,
  normalizeAnglePi,
  shortestAngleDelta,
} from "./geometry.js";

export interface ArmSpec {
  baseX: number;
  baseY: number;
  link1: number;
  link2: number;
  /** Joint angle limits in radians; range width must not exceed 2*PI. */
  theta1Min: number;
  theta1Max: number;
  theta2Min: number;
  theta2Max: number;
}

export type Elbow = "up" | "down";

export interface JointAngles {
  theta1: number;
  theta2: number;
}

export interface Pose {
  x: number;
  y: number;
  /** End-effector orientation in world frame; null means orientation is free. */
  phi: number | null;
}

export interface ArmPoints {
  base: { x: number; y: number };
  elbow: { x: number; y: number };
  tip: { x: number; y: number };
  phi: number;
}

export interface IKSolution extends JointAngles {
  elbow: Elbow;
  singular: boolean;
}

export type IKErrorCode =
  | "UNREACHABLE_TOO_FAR"
  | "UNREACHABLE_TOO_NEAR"
 | "ORIENTATION_INCONSISTENT"
  | "JOINT_LIMIT_VIOLATION";

export class IKError extends Error {
  code: IKErrorCode;
  constructor(code: IKErrorCode, message: string) {
    super(message);
    this.name = "IKError";
    this.code = code;
  }
}

export const EPS = 1e-9;

export function forwardKinematics(arm: ArmSpec, q: JointAngles): ArmPoints {
  const elbowX = arm.baseX + arm.link1 * Math.cos(q.theta1);
  const elbowY = arm.baseY + arm.link1 * Math.sin(q.theta1);
  const phi = q.theta1 + q.theta2;
  const tipX = elbowX + arm.link2 * Math.cos(phi);
  const tipY = elbowY + arm.link2 * Math.sin(phi);
  return {
    base: { x: arm.baseX, y: arm.baseY },
    elbow: { x: elbowX, y: elbowY },
    tip: { x: tipX, y: tipY },
    phi,
  };
}

/**
 * Find a representative of angle `a` (mod 2*PI) that lies within [min, max].
 * The limit interval may wrap the periodic boundary (e.g. [3, 8]).
 * Returns null when no such representative exists.
 */
export function wrapIntoLimit(a: number, min: number, max: number): number | null {
  if (max < min) {
    throw new Error("Invalid joint limit: max < min");
  }
  if (max - min > TAU + EPS) {
    throw new Error("Invalid joint limit: interval wider than 2*PI");
  }
  if (max - min >= TAU - EPS) {
    // Full revolution: return the canonical representative in the interval.
    const candidate = normalizeAnglePi(a);
    return candidate >= min - EPS && candidate <= max + EPS ? candidate : null;
  }
  const base = normalizeAngle0To2Pi(a);
  const kMin = Math.floor((min - base) / TAU) - 1;
  const kMax = Math.ceil((max - base) / TAU) + 1;
  for (let k = kMin; k <= kMax; k++) {
    const candidate = base + k * TAU;
    if (candidate >= min - EPS && candidate <= max + EPS) {
      return candidate;
    }
  }
  return null;
}

export function withinLimits(arm: ArmSpec, q: JointAngles): boolean {
  return (
    wrapIntoLimit(q.theta1, arm.theta1Min, arm.theta1Max) !== null &&
    wrapIntoLimit(q.theta2, arm.theta2Min, arm.theta2Max) !== null
  );
}

interface RawSolution {
  theta1: number;
  theta2: number;
  elbow: Elbow;
  singular: boolean;
}

/**
 * Raw analytical inverse kinematics.
 * When pose.phi is null the end-effector orientation is free and both elbow
 * branches are returned (identical at singular configurations).
 * When pose.phi is fixed, the elbow is fixed geometrically.
 */
export function inverseKinematicsRaw(arm: ArmSpec, pose: Pose): RawSolution[] {
  const dx = pose.x - arm.baseX;
  const dy = pose.y - arm.baseY;
  const r = Math.hypot(dx, dy);
  const l1 = arm.link1;
  const l2 = arm.link2;
  const maxReach = l1 + l2;
  const minReach = Math.abs(l1 - l2);

  if (r > maxReach + EPS) {
    throw new IKError(
      "UNREACHABLE_TOO_FAR",
      `Target distance ${r.toFixed(4)} exceeds max reach ${maxReach.toFixed(4)}`
    );
  }
  if (r < minReach - EPS) {
    throw new IKError(
      "UNREACHABLE_TOO_NEAR",
      `Target distance ${r.toFixed(4)} is below min reach ${minReach.toFixed(4)}`
    );
  }

  if (pose.phi !== null) {
    const phi = pose.phi;
    const elbowX = pose.x - l2 * Math.cos(phi);
    const elbowY = pose.y - l2 * Math.sin(phi);
    const d = distance(arm.baseX, arm.baseY, elbowX, elbowY);
    if (d > l1 + EPS || d < Math.max(0, l1 - 2 * EPS) - EPS) {
      throw new IKError(
        "ORIENTATION_INCONSISTENT",
        "No arm configuration places the elbow at link1 length for that orientation"
      );
    }
    const theta1 = Math.atan2(elbowY - arm.baseY, elbowX - arm.baseX);
    const theta2 = shortestAngleDelta(theta1, phi);
    const singular = Math.abs(shortestAngleDelta(0, theta2)) < 1e-6;
    return [{ theta1, theta2, elbow: "up", singular }];
  }

  const rr = Math.min(Math.max(r, minReach), maxReach);
  let cosTheta2 = (rr * rr - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  cosTheta2 = Math.max(-1, Math.min(1, cosTheta2));
  const singular =
    Math.abs(rr - maxReach) < 1e-6 || Math.abs(rr - minReach) < 1e-6;

  const gamma = Math.atan2(dy, dx);
  const solutions: RawSolution[] = [];
  const branches: Array<[Elbow, number]> = [
    ["up", Math.acos(cosTheta2)],
    ["down", -Math.acos(cosTheta2)],
  ];
  for (const [elbow, theta2] of branches) {
    const theta1 =
      gamma - Math.atan2(l2 * Math.sin(theta2), l1 + l2 * Math.cos(theta2));
    solutions.push({ theta1, theta2, elbow, singular });
  }
  return solutions;
}

export interface IKReport {
  pose: Pose;
  solutions: IKSolution[];
  rejected: Array<{ elbow: Elbow; reason: IKErrorCode; theta1: number; theta2: number }>;
}

/**
 * Full IK: enumerate both elbow branches, then keep every periodic
 * representative that satisfies the joint limits. Duplicate branches at
 * singular poses are merged.
 */
export function inverseKinematics(arm: ArmSpec, pose: Pose): IKReport {
  const raw = inverseKinematicsRaw(arm, pose);
  const solutions: IKSolution[] = [];
  const rejected: IKReport["rejected"] = [];
  const seen = new Set<string>();

  for (const s of raw) {
    const t1 = wrapIntoLimit(s.theta1, arm.theta1Min, arm.theta1Max);
    const t2 = wrapIntoLimit(s.theta2, arm.theta2Min, arm.theta2Max);
    if (t1 === null || t2 === null) {
      rejected.push({
        elbow: s.elbow,
        reason: "JOINT_LIMIT_VIOLATION",
        theta1: s.theta1,
        theta2: s.theta2,
      });
      continue;
    }
    const key = `${Math.round(normalizeAngle0To2Pi(t1) / 1e-9)}:${Math.round(
      normalizeAngle0To2Pi(t2) / 1e-9
    )}`;
    if (!seen.has(key)) {
      seen.add(key);
      solutions.push({ theta1: t1, theta2: t2, elbow: s.elbow, singular: s.singular });
    }
  }
  return { pose, solutions, rejected };
}

/** Verify an IK solution actually reaches the target (round-trip guard). */
export function reachesPose(
  arm: ArmSpec,
  q: JointAngles,
  pose: Pose,
  tol = 1e-6
): boolean {
  const fk = forwardKinematics(arm, q);
  if (Math.hypot(fk.tip.x - pose.x, fk.tip.y - pose.y) > tol) return false;
  if (pose.phi !== null && !anglesEquivalent(fk.phi, pose.phi, tol)) return false;
  return true;
}

/** Linear interpolation in joint space taking shortest signed angle deltas. */
export function interpolateJoints(a: JointAngles, b: JointAngles, t: number): JointAngles {
  return {
    theta1: a.theta1 + shortestAngleDelta(a.theta1, b.theta1) * t,
    theta2: a.theta2 + shortestAngleDelta(a.theta2, b.theta2) * t,
  };
}

export function jointDistance(a: JointAngles, b: JointAngles): number {
  return Math.hypot(
    shortestAngleDelta(a.theta1, b.theta1),
    shortestAngleDelta(a.theta2, b.theta2)
  );
}
