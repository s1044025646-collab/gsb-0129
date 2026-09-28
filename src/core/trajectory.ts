import { radToDeg } from "./geometry.js";
import { forwardKinematics } from "./kinematics.js";
import type { ArmConfig, JointAngles } from "./types.js";

export interface TrajectoryRow {
  index: number;
  theta1Rad: number;
  theta2Rad: number;
  theta1Deg: number;
  theta2Deg: number;
  elbowX: number;
  elbowY: number;
  tipX: number;
  tipY: number;
  phiRad: number;
}

/** Export joint angles and resulting end-effector trajectory. */
export function exportTrajectory(arm: ArmConfig, path: JointAngles[]): TrajectoryRow[] {
  return path.map((q, index) => {
    const fk = forwardKinematics(arm, q);
    return {
      index,
      theta1Rad: q.theta1,
      theta2Rad: q.theta2,
      theta1Deg: radToDeg(q.theta1),
      theta2Deg: radToDeg(q.theta2),
      elbowX: fk.joint.x,
      elbowY: fk.joint.y,
      tipX: fk.tip.x,
      tipY: fk.tip.y,
      phiRad: fk.phi,
    };
  });
}

export function toCsv(rows: TrajectoryRow[]): string {
  const header = [
    "index",
    "theta1_rad",
    "theta2_rad",
    "theta1_deg",
    "theta2_deg",
    "elbow_x",
    "elbow_y",
    "tip_x",
    "tip_y",
    "phi_rad",
  ].join(",");
  const body = rows.map((r) =>
    [
      r.index,
      r.theta1Rad.toFixed(8),
      r.theta2Rad.toFixed(8),
      r.theta1Deg.toFixed(6),
      r.theta2Deg.toFixed(6),
      r.elbowX.toFixed(8),
      r.elbowY.toFixed(8),
      r.tipX.toFixed(8),
      r.tipY.toFixed(8),
      r.phiRad.toFixed(8),
    ].join(","),
  );
  return [header, ...body].join("\n") + "\n";
}

/**
 * Densify a planner path by fixed-step subdivision, producing an executable
 * trajectory with bounded per-step joint motion.
 */
export function densify(path: JointAngles[], maxStep: number): JointAngles[] {
  if (path.length === 0) return [];
  const out: JointAngles[] = [path[0]!];
  for (let i = 1; i < path.length; i++) {
    const a = path[i - 1]!;
    const b = path[i]!;
    const d = Math.hypot(b.theta1 - a.theta1, b.theta2 - a.theta2);
    const n = Math.max(1, Math.ceil(d / maxStep));
    for (let k = 1; k <= n; k++) {
      const t = k / n;
      out.push({
        theta1: a.theta1 + (b.theta1 - a.theta1) * t,
        theta2: a.theta2 + (b.theta2 - a.theta2) * t,
      });
    }
  }
  return out;
}
