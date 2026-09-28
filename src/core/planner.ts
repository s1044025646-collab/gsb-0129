import { checkEdge, configCollision } from "./collision.js";
import { wrapAngle } from "./geometry.js";
import {
  filterByLimits,
  inverseKinematics,
} from "./kinematics.js";
import { Rng } from "./rng.js";
import type {
  ArmConfig,
  CircleObstacle,
  JointAngles,
  PathReport,
  PlanningParams,
  Pose,
} from "./types.js";

export type PlannerErrorCode =
  | "START_UNREACHABLE"
  | "GOAL_UNREACHABLE"
  | "NO_ELBOW_IN_LIMITS"
  | "START_IN_COLLISION"
  | "GOAL_IN_COLLISION"
  | "BUDGET_EXHAUSTED";

export class PlannerError extends Error {
  code: PlannerErrorCode;
  constructor(code: PlannerErrorCode, message: string) {
    super(message);
    this.name = "PlannerError";
    this.code = code;
  }
}

export interface PlannerOptions {
  linkHalfWidth?: number;
  clearance?: number;
  /** Injected clock for deterministic timing in tests. */
  now?: () => number;
}

interface RrtNode {
  q: JointAngles;
  parent: number | null;
}

function jointDistance(a: JointAngles, b: JointAngles): number {
  // Wrapped shortest distance handles periodic boundaries.
  const d1 = Math.abs(wrapAngle(b.theta1 - a.theta1));
  const d2 = Math.abs(wrapAngle(b.theta2 - a.theta2));
  return Math.hypot(d1, d2);
}

function steer(a: JointAngles, b: JointAngles, step: number): JointAngles {
  // Choose periodic equivalents of b nearest to a so motion is short.
  const target: JointAngles = {
    theta1: a.theta1 + wrapAngle(b.theta1 - a.theta1),
    theta2: a.theta2 + wrapAngle(b.theta2 - a.theta2),
  };
  const d = jointDistance(a, target);
  if (d <= step) return target;
  const t = step / d;
  return {
    theta1: a.theta1 + (target.theta1 - a.theta1) * t,
    theta2: a.theta2 + (target.theta2 - a.theta2) * t,
  };
}

/** Choose a feasible IK configuration for a pose under limits and collision. */
export function resolveConfig(
  arm: ArmConfig,
  obstacles: CircleObstacle[],
  pose: Pose,
  opts: PlannerOptions = {},
): { q: JointAngles; elbow: string; collisions: boolean }[] {
  const sols = inverseKinematics(arm, { x: pose.x, y: pose.y });
  const limited = filterByLimits(sols, arm.limits);
  return limited.map((s) => {
    const q = { theta1: s.theta1, theta2: s.theta2 };
    return {
      q,
      elbow: s.elbow,
      collisions: configCollision(arm, obstacles, q, {
        linkHalfWidth: opts.linkHalfWidth,
        clearance: opts.clearance,
      }),
    };
  });
}

export interface PlanResult {
  status: "success" | "no_path";
  path: JointAngles[];
  report: PathReport;
  start: JointAngles;
  goal: JointAngles;
}

/**
 * Joint-space RRT planner for a 2R arm.
 *
 * - Fixed seed => reproducible trees and results.
 * - Every extension edge is densely collision-checked at `resolution`,
 *   preventing tunneling through thin obstacles.
 * - When the budget is exhausted the result is `no_path` with reason
 *   BUDGET_EXHAUSTED — never claimed to be a geometric proof of impossibility.
 */
export function planPath(
  arm: ArmConfig,
  obstacles: CircleObstacle[],
  startPose: Pose,
  goalPose: Pose,
  params: PlanningParams,
  opts: PlannerOptions = {},
): PlanResult {
  const now = opts.now ?? (() => Date.now());
  const t0 = now();

  let startCandidates: ReturnType<typeof resolveConfig>;
  let goalCandidates: ReturnType<typeof resolveConfig>;
  try {
    startCandidates = resolveConfig(arm, obstacles, startPose, opts);
  } catch (e) {
    throw new PlannerError(
      "START_UNREACHABLE",
      `Start pose unreachable: ${(e as Error).message}`,
    );
  }
  try {
    goalCandidates = resolveConfig(arm, obstacles, goalPose, opts);
  } catch (e) {
    throw new PlannerError(
      "GOAL_UNREACHABLE",
      `Goal pose unreachable: ${(e as Error).message}`,
    );
  }

  if (startCandidates.length === 0 || goalCandidates.length === 0) {
    throw new PlannerError(
      "NO_ELBOW_IN_LIMITS",
      "No inverse-kinematics elbow solution satisfies the joint limits",
    );
  }

  const freeStarts = startCandidates.filter((c) => !c.collisions);
  const freeGoals = goalCandidates.filter((c) => !c.collisions);
  const reportBase: PathReport = {
    status: "no_path",
    iterations: 0,
    nodes: 0,
    segments: 0,
    collisions: 0,
    elapsedMs: 0,
    startCollision: freeStarts.length === 0,
    goalCollision: freeGoals.length === 0,
  };

  if (freeStarts.length === 0) {
    return {
      status: "no_path",
      path: [],
      start: startCandidates[0]!.q,
      goal: (freeGoals[0] ?? goalCandidates[0])!.q,
      report: { ...reportBase, reason: "START_IN_COLLISION", elapsedMs: now() - t0 },
    };
  }
  if (freeGoals.length === 0) {
    return {
      status: "no_path",
      path: [],
      start: freeStarts[0]!.q,
      goal: goalCandidates[0]!.q,
      report: { ...reportBase, reason: "GOAL_IN_COLLISION", elapsedMs: now() - t0 },
    };
  }

  // Deterministic branch choice: prefer elbow up, else first free branch.
  const startQ = freeStarts[0]!.q;
  const goalQ = freeGoals[0]!.q;

  const rng = new Rng(params.seed);
  const nodes: RrtNode[] = [{ q: startQ, parent: null }];
  let collisionEdges = 0;

  const sampleFree = (): JointAngles => {
    if (rng.next() < params.goalBias) return goalQ;
    return {
      theta1: rng.uniform(arm.limits.theta1Min, arm.limits.theta1Max),
      theta2: rng.uniform(arm.limits.theta2Min, arm.limits.theta2Max),
    };
  };

  const nearest = (q: JointAngles): number => {
    let best = 0;
    let bestD = Infinity;
    for (let i = 0; i < nodes.length; i++) {
      const d = jointDistance(nodes[i]!.q, q);
      if (d < bestD) {
        bestD = d;
        best = i;
      }
    }
    return best;
  };

  for (let iter = 1; iter <= params.maxIterations; iter++) {
    const target = sampleFree();
    const nearIdx = nearest(target);
    const near = nodes[nearIdx]!.q;
    const newQ = steer(near, target, params.step);

    const edge = checkEdge(
      arm,
      obstacles,
      near,
      newQ,
      params.resolution,
      { linkHalfWidth: opts.linkHalfWidth, clearance: opts.clearance },
    );
    if (!edge.safe) {
      collisionEdges++;
      continue;
    }
    const newIdx = nodes.length;
    nodes.push({ q: newQ, parent: nearIdx });

    // Extend one more step toward the goal; if the extension is collision
    // free under dense sampling and actually reaches the goal, connect.
    const towardGoal = steer(newQ, goalQ, params.step);
    const goalEdge = checkEdge(
      arm,
      obstacles,
      newQ,
      towardGoal,
      params.resolution,
      { linkHalfWidth: opts.linkHalfWidth, clearance: opts.clearance },
    );
    const reached = jointDistance(towardGoal, goalQ) < 1e-6;
    if (goalEdge.safe && reached) {
      // Two-segment connection: newQ -> towardGoal -> goalQ. Each segment is
      // densely collision-checked individually; when towardGoal is already
      // the goal there is just one segment. No segment may skip verification.
      const connectIdx = nodes.length;
      nodes.push({ q: towardGoal, parent: newIdx });
      const tailEdge =
        jointDistance(towardGoal, goalQ) < 1e-9
          ? { safe: true as const }
          : checkEdge(
              arm,
              obstacles,
              towardGoal,
              goalQ,
              params.resolution,
              { linkHalfWidth: opts.linkHalfWidth, clearance: opts.clearance },
            );
      if (tailEdge.safe) {
        if (jointDistance(towardGoal, goalQ) >= 1e-9) {
          nodes.push({ q: goalQ, parent: connectIdx });
        }
        const path = extractPath(nodes, nodes.length - 1);
        return {
          status: "success",
          path,
          start: startQ,
          goal: goalQ,
          report: {
            status: "success",
            iterations: iter,
            nodes: nodes.length,
            segments: path.length - 1,
            collisions: collisionEdges,
            elapsedMs: now() - t0,
            startCollision: false,
            goalCollision: false,
          },
        };
      }
    }
  }

  return {
    status: "no_path",
    path: [],
    start: startQ,
    goal: goalQ,
    report: {
      ...reportBase,
      status: "no_path",
      reason: "BUDGET_EXHAUSTED",
      iterations: params.maxIterations,
      nodes: nodes.length,
      segments: 0,
      collisions: collisionEdges,
      elapsedMs: now() - t0,
      startCollision: false,
      goalCollision: false,
    },
  };
}

function extractPath(nodes: RrtNode[], idx: number): JointAngles[] {
  const path: JointAngles[] = [];
  let cur: number | null = idx;
  while (cur !== null) {
    path.push(nodes[cur]!.q);
    cur = nodes[cur]!.parent;
  }
  path.reverse();
  return path;
}
