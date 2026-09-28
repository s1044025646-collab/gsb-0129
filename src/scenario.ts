import { Circle, TAU, degToRad } from "./geometry.js";
import { ArmSpec, JointAngles, Pose } from "./kinematics.js";
import { mulberry32 } from "./rng.js";

export interface Scenario {
  name: string;
  arm: ArmSpec;
  obstacles: Circle[];
  start: JointAngles;
  goal: JointAngles;
  goalPose: Pose;
  description: string;
}

export type ScenarioKind =
  | "clear"
  | "tipSafeLinkHits"
  | "detour"
  | "startCollision"
  | "narrowPassage";

/** Hand-built and randomly generated fictional scenarios. No real robot data. */
export function generateScenario(kind: ScenarioKind, seed = 1): Scenario {
  const arm: ArmSpec = {
    baseX: 0,
    baseY: 0,
    link1: 2,
    link2: 1.5,
    theta1Min: -Math.PI,
    theta1Max: Math.PI,
    theta2Min: -Math.PI,
    theta2Max: Math.PI,
  };

  switch (kind) {
    case "clear":
      return {
        name: "clear",
        description: "Empty workspace; direct motion.",
        arm,
        obstacles: [],
        start: { theta1: degToRad(20), theta2: degToRad(40) },
        goal: { theta1: degToRad(110), theta2: degToRad(-30) },
        goalPose: { x: 0, y: 0, phi: degToRad(80) },
      };

    case "tipSafeLinkHits": {
      // Both endpoints are collision free, but the direct joint-space motion
      // sweeps link1 through the circle while the tip stays clear of it.
      const o: Circle = { id: "obs1", x: 1.32, y: 0.72, radius: 0.22 };
      return {
        name: "tipSafeLinkHits",
        description: "Tip is clear but link1 sweeps through a circle.",
        arm,
        obstacles: [o],
        start: { theta1: degToRad(5), theta2: degToRad(50) },
        goal: { theta1: degToRad(60), theta2: degToRad(10) },
        goalPose: { x: 1.0, y: 2.0, phi: degToRad(70) },
      };
    }

    case "detour": {
      const o: Circle = { id: "wall", x: 1.5, y: 1.4, radius: 0.45 };
      return {
        name: "detour",
        description: "Central obstacle forces a detour in joint space.",
        arm,
        obstacles: [o],
        start: { theta1: degToRad(10), theta2: degToRad(50) },
        goal: { theta1: degToRad(100), theta2: degToRad(60) },
        goalPose: { x: -0.6, y: 0.2, phi: degToRad(160) },
      };
    }

    case "startCollision": {
      const o: Circle = { id: "atStart", x: 1.7, y: 0.3, radius: 0.3 };
      return {
        name: "startCollision",
        description: "Start configuration already intersects an obstacle.",
        arm,
        obstacles: [o],
        start: { theta1: degToRad(10), theta2: degToRad(10) },
        goal: { theta1: degToRad(120), theta2: degToRad(30) },
        goalPose: { x: -0.5, y: 0, phi: degToRad(150) },
      };
    }

    case "narrowPassage": {
      // Two circles forming a gap the links must thread through.
      const a: Circle = { id: "postA", x: 1.0, y: 1.55, radius: 0.32 };
      const b: Circle = { id: "postB", x: 2.35, y: 1.55, radius: 0.32 };
      return {
        name: "narrowPassage",
        description: "Two posts leave a narrow corridor for the arm.",
        arm,
        obstacles: [a, b],
        start: { theta1: degToRad(5), theta2: degToRad(15) },
        goal: { theta1: degToRad(70), theta2: degToRad(20) },
        goalPose: { x: 0.4, y: 0, phi: degToRad(90) },
      };
    }
  }
}

/** Procedurally generated fictional obstacles; fixed seed => fixed scene. */
export function randomScenario(seed: number): Scenario {
  const rand = mulberry32(seed);
  const count = 2 + Math.floor(rand() * 3);
  const obstacles: Circle[] = [];
  for (let i = 0; i < count; i++) {
    obstacles.push({
      id: `r${i}`,
      x: -1 + rand() * 3,
      y: -1 + rand() * 3,
      radius: 0.15 + rand() * 0.25,
    });
  }
  const arm: ArmSpec = {
    baseX: 0,
    baseY: 0,
    link1: 2,
    link2: 1.5,
    theta1Min: -Math.PI,
    theta1Max: Math.PI,
    theta2Min: -Math.PI,
    theta2Max: Math.PI,
  };
  const rand2 = mulberry32(seed ^ 0x9e3779b9);
  const start: JointAngles = {
    theta1: -0.5 + rand2() * 1,
    theta2: -0.5 + rand2() * 1,
  };
  const goal: JointAngles = {
    theta1: 1 + rand2() * 1.5,
    theta2: -0.5 + rand2() * 1,
  };
  return {
    name: `random-${seed}`,
    description: `Procedurally generated scene (seed ${seed}).`,
    arm,
    obstacles,
    start,
    goal,
    goalPose: { x: 0, y: 0, phi: null },
  };
}
