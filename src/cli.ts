#!/usr/bin/env node
import { mkdirSync, writeFileSync } from "node:fs";
import { join } from "node:path";
import { openDatabase } from "./db.js";
import { Store } from "./store.js";
import { KinematicsService } from "./service.js";
import { generateScenario, randomScenario, ScenarioKind } from "./scenario.js";
import { DEFAULT_PARAMS, PlannerParams } from "./planner.js";
import { degToRad, radToDeg } from "./geometry.js";
import { startServer } from "./server.js";

const DB_PATH = process.env.ARM_DB_PATH ?? join("data", "arm.sqlite");

function fmt(n: number): string {
  return Number.isFinite(n) ? n.toFixed(4) : String(n);
}

function banner(title: string): void {
  console.log(`\n=== ${title} ===`);
}

function parseArgs(argv: string[]): Map<string, string> {
  const map = new Map<string, string>();
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i]!;
    if (a.startsWith("--")) {
      const next = argv[i + 1];
      if (next !== undefined && !next.startsWith("--")) {
        map.set(a.slice(2), next);
        i++;
      } else {
        map.set(a.slice(2), "true");
      }
    }
  }
  return map;
}

async function main(): Promise<void> {
  const [command, ...rest] = process.argv.slice(2);
  const args = parseArgs(rest);

  if (!command || command === "help" || command === "--help") {
    printHelp();
    return;
  }

  if (command === "serve") {
    const port = Number(args.get("port") ?? process.env.PORT ?? "0");
    const { port: actual } = await startServer(DB_PATH, port);
    console.log(`API listening on http://127.0.0.1:${actual} (loopback only)`);
    console.log(`Database: ${DB_PATH}`);
    console.log("Press Ctrl+C to stop.");
    return;
  }

  mkdirSync("data", { recursive: true });
  const db = openDatabase(DB_PATH);
  const store = new Store(db);
  const service = new KinematicsService(store);

  switch (command) {
    case "create-scenario": {
      const kind = (args.get("kind") ?? "clear") as ScenarioKind;
      const seedArg = args.get("seed");
      const scenario = seedArg ? randomScenario(Number(seedArg)) : generateScenario(kind);
      const id = args.get("id") ?? scenario.name;
      const before = store.listScenarios(1, 0).total;
      banner(`BEFORE create-scenario: ${before} scenario(s) in DB`);
      const saved = store.upsertScenario({
        id,
        name: scenario.name,
        description: scenario.description,
        arm: scenario.arm,
        obstacles: scenario.obstacles,
        start: scenario.start,
        goal: scenario.goal,
        goalPose: scenario.goalPose,
      });
      const after = store.listScenarios(1, 0).total;
      console.log(`Created scenario id=${saved.id} kind=${kind} hash=${saved.hash.slice(0, 12)}`);
      console.log(
        `  links L1=${saved.arm.link1} L2=${saved.arm.link2}, obstacles=${saved.obstacles.length}`
      );
      console.log(
        `  start=(${fmt(radToDeg(saved.start.theta1))}deg, ${fmt(radToDeg(saved.start.theta2))}deg)`
      );
      console.log(
        `  goal =(${fmt(radToDeg(saved.goal.theta1))}deg, ${fmt(radToDeg(saved.goal.theta2))}deg)`
      );
      banner(`AFTER create-scenario: ${after} scenario(s) in DB`);
      break;
    }

    case "list-scenarios": {
      const page = store.listScenarios(50, 0);
      banner(`Scenarios (${page.total})`);
      for (const s of page.items) {
        console.log(`- ${s.id} | ${s.name} | hash=${s.hash.slice(0, 8)} | obs=${s.obstacles.length}`);
      }
      break;
    }

    case "fk": {
      const scenarioId = args.get("scenario") ?? "clear";
      const s = requireScenario(store, scenarioId);
      const q = {
        theta1: degToRad(Number(args.get("theta1-deg") ?? 20)),
        theta2: degToRad(Number(args.get("theta2-deg") ?? 40)),
      };
      const pts = service.fk(s.arm, q);
      banner("Forward Kinematics");
      console.log(`  base   = (${fmt(pts.base.x)}, ${fmt(pts.base.y)})`);
      console.log(`  elbow  = (${fmt(pts.elbow.x)}, ${fmt(pts.elbow.y)})`);
      console.log(`  tip    = (${fmt(pts.tip.x)}, ${fmt(pts.tip.y)})`);
      console.log(`  phi    = ${fmt(radToDeg(pts.phi))}deg`);
      break;
    }

    case "ik": {
      const scenarioId = args.get("scenario") ?? "clear";
      const s = requireScenario(store, scenarioId);
      const pose = {
        x: Number(args.get("x") ?? s.goalPose.x),
        y: Number(args.get("y") ?? s.goalPose.y),
        phi: args.has("phi-deg") ? degToRad(Number(args.get("phi-deg"))) : null,
      };
      banner(`Inverse Kinematics target=(${fmt(pose.x)}, ${fmt(pose.y)})`);
      try {
        const report = service.ik(s.arm, pose);
        console.log(`  feasible solutions: ${report.solutions.length}`);
        for (const sol of report.solutions) {
          console.log(
            `  * elbow=${sol.elbow} t1=${fmt(radToDeg(sol.theta1))}deg t2=${fmt(
              radToDeg(sol.theta2)
            )}deg singular=${sol.singular}`
          );
        }
        for (const rej of report.rejected) {
          console.log(`  - rejected elbow=${rej.elbow} reason=${rej.reason}`);
        }
      } catch (err) {
        console.log(`  IK failed: ${(err as Error).message}`);
      }
      break;
    }

    case "plan": {
      const scenarioId = args.get("scenario") ?? "clear";
      const s = requireScenario(store, scenarioId);
      const params = paramsFromArgs(args);
      const before = store.listPlans(s.id, 1, 0).total;
      banner(`BEFORE plan: scenario '${s.id}' has ${before} stored plan(s)`);
      const { record, reused } = service.plan(s, params, args.get("idempotency-key"));
      console.log(
        `status=${record.status} reused=${reused} iterations=${record.iterationsUsed}/${record.budget}`
      );
      console.log(
        `waypoints=${record.path.length} trajectoryPoints=${record.trajectory.length} valid=${record.valid}`
      );
      if (record.invalidReason) console.log(`invalidReason=${record.invalidReason}`);
      const after = store.listPlans(s.id, 1, 0).total;
      banner(`AFTER plan: scenario '${s.id}' has ${after} stored plan(s); planId=${record.id}`);
      break;
    }

    case "verify": {
      const plan = requireLatestPlan(store, args);
      banner(`BEFORE verify: plan ${plan.id} status=${plan.status} valid=${plan.valid}`);
      const v = service.verifyPlan(plan);
      console.log(
        `segments=${v.segmentCount} ok=${v.ok} failures=${v.failures.length} minClearance=${fmt(
          v.minClearance
        )}`
      );
      for (const f of v.failures) {
        console.log(
          `  collision segment=${f.index} t=${fmt(f.t)} obstacle=${f.obstacleId} link=${f.link}`
        );
      }
      banner(`AFTER verify: report ok=${v.ok}`);
      break;
    }

    case "compare": {
      const scenarioId = args.get("scenario") ?? "clear";
      const s = requireScenario(store, scenarioId);
      const a: PlannerParams = { ...DEFAULT_PARAMS, seed: Number(args.get("seed-a") ?? 1) };
      const b: PlannerParams = {
        ...DEFAULT_PARAMS,
        seed: Number(args.get("seed-b") ?? 2),
        maxIterations: Number(args.get("budget-b") ?? 500),
      };
      banner("Compare parameter sets A vs B");
      const cmp = service.compareParams(s, a, b);
      console.log(`A: ${JSON.stringify(cmp.a)}`);
      console.log(`B: ${JSON.stringify(cmp.b)}`);
      console.log(`sameStatus=${cmp.sameStatus} samePathLength=${cmp.samePathLength}`);
      break;
    }

    case "export": {
      const plan = requireLatestPlan(store, args);
      const traj = service.exportTrajectory(plan);
      const out = args.get("out") ?? join("data", `trajectory-${plan.id.slice(0, 8)}.json`);
      writeFileSync(out, JSON.stringify({ planId: plan.id, ...traj }, null, 2));
      console.log(`Exported ${traj.joints.length} points to ${out}`);
      break;
    }

    case "list-plans": {
      const scenarioId = args.get("scenario") ?? null;
      const page = store.listPlans(scenarioId, 50, 0);
      banner(`Plans (${page.total})`);
      for (const p of page.items) {
        console.log(
          `- ${p.id} scenario=${p.scenarioId} status=${p.status} valid=${p.valid} v=${p.paramsVersion}`
        );
      }
      break;
    }

    default:
      console.error(`Unknown command: ${command}`);
      printHelp();
      process.exitCode = 1;
  }
  db.close();
}

function paramsFromArgs(args: Map<string, string>): PlannerParams {
  return {
    ...DEFAULT_PARAMS,
    seed: Number(args.get("seed") ?? DEFAULT_PARAMS.seed),
    maxIterations: Number(args.get("budget") ?? DEFAULT_PARAMS.maxIterations),
    step: Number(args.get("step") ?? DEFAULT_PARAMS.step),
    paramsVersion: args.get("params-version") ?? DEFAULT_PARAMS.paramsVersion,
  };
}

function requireScenario(store: Store, id: string) {
  const s = store.getScenario(id);
  if (!s) {
    console.error(`Scenario '${id}' not found. Run: cli create-scenario --id ${id}`);
    process.exit(1);
  }
  return s;
}

function requireLatestPlan(store: Store, args: Map<string, string>) {
  const id = args.get("plan");
  const plan = id
    ? store.getPlan(id)
    : store.listPlans(args.get("scenario") ?? null, 1, 0).items[0] ?? null;
  if (!plan) {
    console.error("No plan found. Run 'plan' first or pass --plan <plan-id>.");
    process.exit(1);
  }
  return store.refreshPlanValidity(plan.id)!;
}

function printHelp(): void {
  console.log(
    [
      "Planar 2-DOF arm kinematics backend CLI",
      "",
      "Usage:",
      "  serve [--port 0]",
      "  create-scenario [--id ID] [--kind clear|tipSafeLinkHits|detour|startCollision|narrowPassage] [--seed N]",
      "  list-scenarios",
      "  fk --scenario ID [--theta1-deg D] [--theta2-deg D]",
      "  ik --scenario ID [--x X] [--y Y] [--phi-deg D]",
      "  plan --scenario ID [--seed N] [--budget N] [--step S] [--params-version V] [--idempotency-key K]",
      "  verify [--plan ID | --scenario ID]",
      "  compare --scenario ID [--seed-a N] [--seed-b N] [--budget-b N]",
      "  export [--plan ID | --scenario ID] [--out FILE]",
      "  list-plans [--scenario ID]",
      "",
      "Data stays under ./data. Server binds 127.0.0.1 only; port 0 = auto-assigned.",
    ].join("\n")
  );
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
