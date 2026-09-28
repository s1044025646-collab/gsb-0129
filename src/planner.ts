import { Circle, shortestAngleDelta } from "./geometry.js";
import {
  ArmSpec,
  JointAngles,
  interpolateJoints,
  jointDistance,
  withinLimits,
} from "./kinematics.js";
import { checkMotionCollision, checkPoseCollision } from "./collision.js";
import { mulberry32 } from "./rng.js";

export interface PlannerParams {
  /** RNG seed; same seed + same inputs always produce the same path. */
  seed: number;
  /** Maximum RRT extension iterations before declaring no path found. */
  maxIterations: number;
  /** Fixed joint-space extension step (radians). */
  step: number;
  /** Goal sampling probability (0..1); remaining samples are uniform. */
  goalBias: number;
  /** Obstacle inflation in world units during planning and verification. */
  safetyMargin: number;
  /** Maximum cartesian motion per collision substep. */
  maxCartesianStep: number;
  /** Version tag identifying this parameter set. */
  paramsVersion: string;
}

export const DEFAULT_PARAMS: PlannerParams = {
  seed: 12345,
  maxIterations: 2000,
  step: 0.25,
  goalBias: 0.12,
  safetyMargin: 0.01,
  maxCartesianStep: 0.04,
  paramsVersion: "v1",
};

export type PlanStatus = "FOUND" | "NOT_FOUND" | "START_IN_COLLISION" | "GOAL_IN_COLLISION";

export interface PlanResult {
  status: PlanStatus;
  paramsVersion: string;
  seed: number;
  iterationsUsed: number;
  budget: number;
  /** Waypoints in joint space, including start and goal. */
  path: JointAngles[];
  /** Dense interpolated trajectory after pruning. */
  trajectory: JointAngles[];
  collisionObstacleId?: string;
  collisionLink?: "link1" | "link2" | "base";
}

interface TreeNode {
  q: JointAngles;
  parent: number | null;
}

function randomConfig(arm: ArmSpec, rand: () => number): JointAngles {
  return {
    theta1: arm.theta1Min + rand() * (arm.theta1Max - arm.theta1Min),
    theta2: arm.theta2Min + rand() * (arm.theta2Max - arm.theta2Min),
  };
}

function nearest(nodes: TreeNode[], q: JointAngles): number {
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
}

function steer(from: JointAngles, toward: JointAngles, step: number): JointAngles {
  const d1 = shortestAngleDelta(from.theta1, toward.theta1);
  const d2 = shortestAngleDelta(from.theta2, toward.theta2);
  const d = Math.hypot(d1, d2);
  if (d <= step) return { ...toward };
  const t = step / d;
  return { theta1: from.theta1 + d1 * t, theta2: from.theta2 + d2 * t };
}

function collisionFreeMotion(
  arm: ArmSpec,
  a: JointAngles,
  b: JointAngles,
  obstacles: Circle[],
  p: PlannerParams
): boolean {
  return (
    checkMotionCollision(arm, a, b, obstacles, {
      maxStep: p.maxCartesianStep,
      safetyMargin: p.safetyMargin,
    }) === null
  );
}

function extractPath(nodes: TreeNode[], idx: number): JointAngles[] {
  const path: JointAngles[] = [];
  let cur: number | null = idx;
  while (cur !== null) {
    path.push(nodes[cur]!.q);
    cur = nodes[cur]!.parent;
  }
  path.reverse();
  return path;
}

/** Greedy pruning: drop intermediate waypoints whenever a direct motion is safe. */
function prunePath(
  arm: ArmSpec,
  path: JointAngles[],
  obstacles: Circle[],
  p: PlannerParams
): JointAngles[] {
  if (path.length <= 2) return path;
  const result: JointAngles[] = [path[0]!];
  let anchor = 0;
  while (anchor < path.length - 1) {
    let reach = anchor + 1;
    for (let j = path.length - 1; j > anchor + 1; j--) {
      if (collisionFreeMotion(arm, path[anchor]!, path[j]!, obstacles, p)) {
        reach = j;
        break;
      }
    }
    result.push(path[reach]!);
    anchor = reach;
  }
  return result;
}

function densify(
  arm: ArmSpec,
  path: JointAngles[],
  obstacles: Circle[],
  p: PlannerParams
): JointAngles[] {
  const out: JointAngles[] = [path[0]!];
  for (let i = 0; i < path.length - 1; i++) {
    const a = path[i]!;
    const b = path[i + 1]!;
    const dist = jointDistance(a, b);
    const n = Math.max(1, Math.ceil(dist / p.step));
    for (let k = 1; k <= n; k++) {
      out.push(interpolateJoints(a, b, k / n));
    }
  }
  return out;
}

export interface PlanInput {
  arm: ArmSpec;
  obstacles: Circle[];
  start: JointAngles;
  goal: JointAngles;
}

export function planPath(input: PlanInput, params: PlannerParams): PlanResult {
  const { arm, obstacles, start, goal } = input;
  const base: Omit<PlanResult, "status" | "path" | "trajectory" | "iterationsUsed"> = {
    paramsVersion: params.paramsVersion,
    seed: params.seed,
    budget: params.maxIterations,
  };

  if (!withinLimits(arm, start) || !withinLimits(arm, goal)) {
    return { ...base, status: "NOT_FOUND", iterationsUsed: 0, path: [], trajectory: [] };
  }

  const startHit = checkPoseCollision(arm, start, obstacles, params.safetyMargin);
  if (startHit.collision) {
    return {
      ...base,
      status: "START_IN_COLLISION",
      iterationsUsed: 0,
      path: [],
      trajectory: [],
      collisionObstacleId: startHit.obstacleId,
      collisionLink: startHit.link,
    };
  }
  const goalHit = checkPoseCollision(arm, goal, obstacles, params.safetyMargin);
  if (goalHit.collision) {
    return {
      ...base,
      status: "GOAL_IN_COLLISION",
      iterationsUsed: 0,
      path: [],
      trajectory: [],
      collisionObstacleId: goalHit.obstacleId,
      collisionLink: goalHit.link,
    };
  }

  if (collisionFreeMotion(arm, start, goal, obstacles, params)) {
    const path = [start, goal];
    return {
      ...base,
      status: "FOUND",
      iterationsUsed: 0,
      path,
      trajectory: densify(arm, path, obstacles, params),
    };
  }

  const rand = mulberry32(params.seed);
  let treeA: TreeNode[] = [{ q: start, parent: null }];
  let treeB: TreeNode[] = [{ q: goal, parent: null }];

  for (let iter = 1; iter <= params.maxIterations; iter++) {
    const sample: JointAngles =
      rand() < params.goalBias ? goal : randomConfig(arm, rand);

    // Extend treeA one step toward the sample.
    const nearAIdx = nearest(treeA, sample);
    const nearA = treeA[nearAIdx]!.q;
    const newA = steer(nearA, sample, params.step);
    if (!withinLimits(arm, newA)) continue;
    if (!collisionFreeMotion(arm, nearA, newA, obstacles, params)) continue;
    const newAIdx = treeA.length;
    treeA.push({ q: newA, parent: nearAIdx });

    // Greedily connect treeB toward the newly added node.
    let connectIdx: number = nearest(treeB, newA);
    let cursorIdx: number = connectIdx;
    let cursor = treeB[connectIdx]!.q;
    let guard = 0;
    let connected = false;
    while (guard++ < 1000) {
      const next = steer(cursor, newA, params.step);
      if (!withinLimits(arm, next)) break;
      if (!collisionFreeMotion(arm, cursor, next, obstacles, params)) break;
      const nextIdx = treeB.length;
      treeB.push({ q: next, parent: cursorIdx });
      if (jointDistance(next, newA) <= 1e-9) {
        connected = true;
        connectIdx = nextIdx;
        break;
      }
      cursorIdx = nextIdx;
      cursor = next;
    }

    if (connected) {
      const partA = extractPath(treeA, newAIdx);
      const partB = extractPath(treeB, connectIdx);
      const raw = [...partA, ...partB.slice().reverse()];
      const path = prunePath(arm, raw, obstacles, params);
      return {
        ...base,
        status: "FOUND",
        iterationsUsed: iter,
        path,
        trajectory: densify(arm, path, obstacles, params),
      };
    }

    // Swap trees so the search expands from both ends.
    [treeA, treeB] = [treeB, treeA];
  }

  return {
    ...base,
    status: "NOT_FOUND",
    iterationsUsed: params.maxIterations,
    path: [],
    trajectory: [],
  };
}
