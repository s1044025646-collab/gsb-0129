export type Vec2 = { x: number; y: number };

export interface LinkSpec {
  l1: number;
  l2: number;
}

export interface BaseSpec {
  x: number;
  y: number;
}

/** Joint limits in radians. Interval is interpreted on the real line after
 * adding multiples of 2*pi to the angle (periodic wrapping). */
export interface JointLimits {
  theta1Min: number;
  theta1Max: number;
  theta2Min: number;
  theta2Max: number;
}

export interface CircleObstacle {
  id: string;
  x: number;
  y: number;
  radius: number;
}

export interface Pose {
  x: number;
  y: number;
  /** End-effector orientation in radians (world frame). */
  phi: number;
}

export interface JointAngles {
  theta1: number;
  theta2: number;
}

export interface PlanningParams {
  /** Schema version of the planning parameters, e.g. "v1". */
  version: string;
  /** RRT step length in radians of joint space. */
  step: number;
  /** Max collision-check resolution along a local edge (radians). */
  resolution: number;
  /** Max RRT node expansion attempts. */
  maxIterations: number;
  /** Goal sampling probability in [0,1]. */
  goalBias: number;
  /** Deterministic PRNG seed. */
  seed: number;
}

export interface ArmConfig extends LinkSpec {
  base: BaseSpec;
  limits: JointLimits;
}

export interface ScenarioSpec {
  name: string;
  arm: ArmConfig;
  obstacles: CircleObstacle[];
  start: Pose;
  goal: Pose;
}

export type PlanStatus = "success" | "no_path";

export interface PathReport {
  status: PlanStatus;
  reason?: string;
  iterations: number;
  nodes: number;
  segments: number;
  collisions: number;
  elapsedMs: number;
  startCollision: boolean;
  goalCollision: boolean;
}

export interface StoredPlan {
  id: string;
  scenarioId: string;
  idempotencyKey: string | null;
  params: PlanningParams;
  configStart: JointAngles;
  configGoal: JointAngles;
  path: JointAngles[];
  report: PathReport;
  valid: number;
  createdAt: string;
  invalidatedAt: string | null;
}
