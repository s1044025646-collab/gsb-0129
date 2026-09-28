import { degToRad } from "./geometry.js";
import type { ScenarioSpec } from "./types.js";

/**
 * Fictional scenario generator. All content is synthetic; no real robot
 * hardware is represented.
 *
 * Named presets exercise the tricky planner cases:
 * - open:          no obstacles
 * - detour:        obstacle between start and goal (tip can pass, links can't)
 * - link_collision:end-effector clear, but a link sweeps a circle
 * - start_blocked: start configuration collides
 * - narrow:        gap between two circles forming a narrow passage
 */
export type PresetName = "open" | "detour" | "link_collision" | "start_blocked" | "narrow";

export function generateScenario(preset: PresetName): ScenarioSpec {
  const arm = {
    l1: 1.0,
    l2: 0.8,
    base: { x: 0, y: 0 },
    limits: {
      theta1Min: degToRad(-170),
      theta1Max: degToRad(170),
      theta2Min: degToRad(-160),
      theta2Max: degToRad(160),
    },
  };
  const base: ScenarioSpec = {
    name: preset,
    arm,
    obstacles: [],
    start: { x: 1.3, y: 0.6, phi: 0.6 },
    goal: { x: 1.3, y: -0.6, phi: -0.6 },
  };

  switch (preset) {
    case "open":
      return base;
    case "detour":
      return {
        ...base,
        obstacles: [{ id: "O1", x: 1.15, y: 0.0, radius: 0.18 }],
      };
    case "link_collision":
      // End-effectors stay clear of the circle, but straight joint motion
      // drives the first link through it.
      return {
        ...base,
        start: { x: 0.5, y: 1.15, phi: degToRad(60) },
        goal: { x: 0.5, y: -1.15, phi: degToRad(-60) },
        obstacles: [{ id: "L1", x: 0.55, y: 0.0, radius: 0.22 }],
      };
    case "start_blocked":
      return {
        ...base,
        obstacles: [{ id: "S1", x: 1.3, y: 0.6, radius: 0.15 }],
      };
    case "narrow":
      return {
        ...base,
        start: { x: 1.35, y: 0.55, phi: degToRad(35) },
        goal: { x: 1.35, y: -0.55, phi: degToRad(-35) },
        obstacles: [
          { id: "N1", x: 1.02, y: 0.22, radius: 0.16 },
          { id: "N2", x: 1.02, y: -0.22, radius: 0.16 },
        ],
      };
  }
}

export const PRESETS: PresetName[] = ["open", "detour", "link_collision", "start_blocked", "narrow"];
