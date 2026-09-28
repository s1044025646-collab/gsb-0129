import { rmSync } from "node:fs";
import { join } from "node:path";
import { openDatabase } from "./db.js";
import { Store } from "./store.js";
import { KinematicsService } from "./service.js";
import { generateScenario, ScenarioKind } from "./scenario.js";
import { DEFAULT_PARAMS, PlannerParams } from "./planner.js";
import { radToDeg } from "./geometry.js";

const dbPath = join("data", "demo.sqlite");
rmSync(dbPath, { force: true });

const db = openDatabase(dbPath);
const store = new Store(db, () => new Date("2026-01-01T00:00:00Z"));
const service = new KinematicsService(store);

function line(s: string): void {
  console.log(s);
}

const kinds: ScenarioKind[] = [
  "clear",
  "tipSafeLinkHits",
  "detour",
  "startCollision",
  "narrowPassage",
];

line("REPEATABLE END-TO-END DEMO (frozen clock, fixed seeds)\n");
for (const kind of kinds) {
  const gen = generateScenario(kind);
  const scenario = store.upsertScenario({
    id: kind,
    name: gen.name,
    description: gen.description,
    arm: gen.arm,
    obstacles: gen.obstacles,
    start: gen.start,
    goal: gen.goal,
    goalPose: gen.goalPose,
  });

  const params: PlannerParams = { ...DEFAULT_PARAMS, seed: 42 };
  const first = service.plan(scenario, params, `demo-${kind}`);
  const second = service.plan(scenario, params, `demo-${kind}`); // idempotent
  const verification = service.verifyPlan(first.record);

  line(`[${kind}] ${scenario.description}`);
  line(
    `  plan: status=${first.record.status} iters=${first.record.iterationsUsed}/${first.record.budget} ` +
      `waypoints=${first.record.path.length} points=${first.record.trajectory.length}`
  );
  line(
    `  idempotent replay reused=${second.reused} sameId=${second.record.id === first.record.id}`
  );
  line(
    `  verification ok=${verification.ok} segments=${verification.segmentCount} ` +
      `minClearance=${verification.minClearance.toFixed(3)}`
  );

  if (first.record.status === "FOUND") {
    const exported = service.exportTrajectory(first.record);
    const last = exported.cartesian.at(-1)!;
    line(
      `  exported tip end=(${last.x.toFixed(3)}, ${last.y.toFixed(3)}) phi=${last.phiDeg.toFixed(1)}deg`
    );
  }
  line("");
}

// Demonstrate invalidation after geometry change.
const detour = store.getScenario("detour")!;
const planBefore = service.plan(detour, { ...DEFAULT_PARAMS, seed: 7 }, "demo-invalidation");
line(`[invalidation] plan valid immediately=${planBefore.record.valid}`);
const changed = generateScenario("detour");
changed.obstacles.push({ id: "new", x: 0, y: 2, radius: 0.5 });
store.upsertScenario({
  id: "detour",
  name: changed.name,
  description: changed.description,
  arm: changed.arm,
  obstacles: changed.obstacles,
  start: changed.start,
  goal: changed.goal,
  goalPose: changed.goalPose,
});
const refreshed = store.refreshPlanValidity(planBefore.record.id)!;
line(
  `[invalidation] after obstacle added valid=${refreshed.valid} reason=${refreshed.invalidReason}`
);
line(`[invalidation] snapshot still has ${refreshed.inputSnapshot.obstacles.length} obstacle(s)`);

// Budget exhaustion reports NOT_FOUND, never a collision path or "infeasible".
const tight: PlannerParams = { ...DEFAULT_PARAMS, seed: 99, maxIterations: 5, step: 0.05 };
const narrow = store.getScenario("narrowPassage")!;
const exhausted = service.plan(narrow, tight, "demo-budget");
line(
  `[budget] tiny budget => status=${exhausted.record.status} pathLen=${exhausted.record.path.length} ` +
    `(NOT_FOUND means search budget exhausted, not geometric impossibility)`
);

line(`\nDemo database written to ${dbPath}`);
db.close();
