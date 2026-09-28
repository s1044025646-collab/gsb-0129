import { mkdirSync, rmSync } from "node:fs";
import { join } from "node:path";
import { openDatabase } from "../db/database.js";
import { PlanningService, type Clock } from "../service/planningService.js";
import { defaultParams } from "../core/validation.js";
import { generateScenario } from "../core/generator.js";

/**
 * Repeatable end-to-end demo: every scenario is planned with a fixed seed and
 * injected clock, so printed results are identical run-to-run. Uses an
 * isolated database under tmp/ and leaves all artifacts in the project.
 */
export async function runDemo(): Promise<void> {
  const dir = join(process.cwd(), "tmp", "demo");
  rmSync(dir, { recursive: true, force: true });
  mkdirSync(dir, { recursive: true });

  let tick = 0;
  const clock: Clock = {
    now: () => tick++ * 2,
    iso: () => new Date(Date.UTC(2026, 0, 1, 0, 0, 0) + tick * 1000).toISOString(),
  };

  const db = openDatabase(join(dir, "demo.db"));
  const service = new PlanningService(db, clock);

  const presets = ["open", "detour", "link_collision", "start_blocked", "narrow"] as const;
  for (const name of presets) {
    const spec = generateScenario(name);
    const scenario = service.createScenario(spec.name, spec);
    const resultA = service.plan(scenario.id, defaultParams(42));
    const resultB = service.plan(scenario.id, defaultParams(42), `demo-${name}`);
    const resultC = service.plan(scenario.id, defaultParams(42), `demo-${name}`);
    const verify = resultA.plan.report.status === "success"
      ? service.verify(resultA.plan.id)
      : null;
    const samePath = resultB.reused === false && resultC.reused === true && resultC.plan.id === resultB.plan.id;
    console.log(
      [
        `scenario=${name.padEnd(15)}`,
        `status=${resultA.plan.report.status}`,
        `reason=${resultA.plan.report.reason ?? "-"}`,
        `segments=${resultA.plan.report.segments}`,
        `iterations=${resultA.plan.report.iterations}`,
        `idempotent=${samePath}`,
        `verify=${verify ? verify.validation.ok : "-"}`,
      ].join("  "),
    );
  }

  // Show invalidation on obstacle change.
  const openSpec = generateScenario("open");
  const sc = service.createScenario("invalidation-demo", openSpec);
  const planned = service.plan(sc.id, defaultParams(7));
  console.log(`invalidation-demo plan valid=${planned.plan.valid} status=${planned.plan.report.status}`);
  const changed = structuredClone(openSpec);
  changed.obstacles = [{ id: "WALL", x: 1.0, y: 0, radius: 0.3 }];
  service.updateScenario(sc.id, { spec: changed });
  const after = service.plans.get(planned.plan.id);
  console.log(`after obstacle edit: valid=${after!.valid === 1} invalidatedAt=${after!.invalidated_at}`);
  const recheck = service.verify(planned.plan.id);
  console.log(`re-verify old path against new geometry: ok=${recheck.validation.ok}`);

  db.close();
  console.log(`demo artifacts in ${dir}`);
}

import { pathToFileURL } from "node:url";

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  runDemo().catch((e) => {
    console.error(e);
    process.exit(1);
  });
}
