import { radToDeg } from "./geometry.js";
import {
  ArmSpec,
  IKError,
  JointAngles,
  Pose,
  forwardKinematics,
  inverseKinematics,
  reachesPose,
} from "./kinematics.js";
import { checkMotionCollision } from "./collision.js";
import {
  DEFAULT_PARAMS,
  PlannerParams,
  PlanResult,
  planPath,
} from "./planner.js";
import { Circle } from "./geometry.js";
import { ScenarioRecord, Store, PlanRecord } from "./store.js";

export interface SegmentVerification {
  ok: boolean;
  segmentCount: number;
  failures: Array<{
    index: number;
    t: number;
    obstacleId: string;
    link: string;
    clearance: number;
  }>;
  minClearance: number;
}

export class KinematicsService {
  constructor(private store: Store) {}

  fk(arm: ArmSpec, q: JointAngles) {
    return forwardKinematics(arm, q);
  }

  ik(arm: ArmSpec, pose: Pose) {
    return inverseKinematics(arm, pose);
  }

  plan(
    scenario: ScenarioRecord,
    params: PlannerParams = DEFAULT_PARAMS,
    clientRequestId?: string
  ): { record: PlanRecord; reused: boolean } {
    const result: PlanResult = planPath(
      {
        arm: scenario.arm,
        obstacles: scenario.obstacles,
        start: scenario.start,
        goal: scenario.goal,
      },
      params
    );
    return this.store.insertPlan({
      clientRequestId,
      scenarioId: scenario.id,
      result,
      params,
      snapshot: {
        arm: scenario.arm,
        obstacles: scenario.obstacles,
        start: scenario.start,
        goal: scenario.goal,
        goalPose: scenario.goalPose,
        params,
      },
    });
  }

  /** Re-verify every interpolated segment of a stored path independently. */
  verifyPlan(plan: PlanRecord): SegmentVerification {
    const snap = plan.inputSnapshot;
    const traj =
      plan.trajectory.length > 0
        ? plan.trajectory
        : plan.report.trajectory;
    const failures: SegmentVerification["failures"] = [];
    let minClearance = Infinity;
    for (let i = 0; i < traj.length - 1; i++) {
      const hit = checkMotionCollision(
        snap.arm,
        traj[i]!,
        traj[i + 1]!,
        snap.obstacles,
        {
          maxStep: snap.params.maxCartesianStep,
          safetyMargin: 0,
        }
      );
      if (hit) {
        failures.push({ index: i, ...hit });
      }
      if (hit && hit.clearance < minClearance) minClearance = hit.clearance;
    }
    if (minClearance === Infinity) minClearance = 0;
    return {
      ok: failures.length === 0,
      segmentCount: Math.max(0, traj.length - 1),
      failures,
      minClearance,
    };
  }

  /** Compare planning outcomes under two parameter sets (same scenario). */
  compareParams(
    scenario: ScenarioRecord,
    paramsA: PlannerParams,
    paramsB: PlannerParams
  ) {
    const runA = planPath(
      {
        arm: scenario.arm,
        obstacles: scenario.obstacles,
        start: scenario.start,
        goal: scenario.goal,
      },
      paramsA
    );
    const runB = planPath(
      {
        arm: scenario.arm,
        obstacles: scenario.obstacles,
        start: scenario.start,
        goal: scenario.goal,
      },
      paramsB
    );
    return {
      a: summarize(runA),
      b: summarize(runB),
      sameStatus: runA.status === runB.status,
      samePathLength: runA.trajectory.length === runB.trajectory.length,
    };
  }

  exportTrajectory(plan: PlanRecord): {
    joints: Array<{ t: number; theta1Deg: number; theta2Deg: number }>;
    cartesian: Array<{ t: number; x: number; y: number; phiDeg: number }>;
  } {
    const snap = plan.inputSnapshot;
    const traj = plan.trajectory;
    const n = Math.max(1, traj.length - 1);
    return {
      joints: traj.map((q, i) => ({
        t: i / n,
        theta1Deg: radToDeg(q.theta1),
        theta2Deg: radToDeg(q.theta2),
      })),
      cartesian: traj.map((q, i) => {
        const p = forwardKinematics(snap.arm, q);
        return { t: i / n, x: p.tip.x, y: p.tip.y, phiDeg: radToDeg(p.phi) };
      }),
    };
  }
}

function summarize(r: PlanResult) {
  return {
    status: r.status,
    iterationsUsed: r.iterationsUsed,
    budget: r.budget,
    waypoints: r.path.length,
    trajectoryPoints: r.trajectory.length,
    seed: r.seed,
    paramsVersion: r.paramsVersion,
  };
}

export { IKError, reachesPose };
export type { Circle };
