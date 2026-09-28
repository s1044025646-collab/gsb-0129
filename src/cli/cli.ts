#!/usr/bin/env node
import { writeFileSync, mkdirSync, readFileSync, existsSync } from "node:fs";
import { isAbsolute, join } from "node:path";
import { openDatabase } from "../db/database.js";
import { PlanningService } from "../service/planningService.js";
import { defaultParams } from "../core/validation.js";
import { generateScenario, PRESETS, type PresetName } from "../core/generator.js";
import { radToDeg } from "../core/geometry.js";
import { startApi } from "../api/server.js";

const DATA_DIR = join(process.cwd(), "data");
const DB_PATH = join(DATA_DIR, "arm.db");
const RUN_DIR = join(process.cwd(), "tmp");
const PID_FILE = join(RUN_DIR, "server.pid");
const PORT_FILE = join(RUN_DIR, "server.port");

function getDb() {
  mkdirSync(DATA_DIR, { recursive: true });
  const db = openDatabase(DB_PATH);
  return { db, service: new PlanningService(db) };
}

function fmt(n: number): string {
  return Number(n).toFixed(5);
}

function usage(): void {
  console.log(`planar-arm-planner CLI

Usage:
  arm init-scenario [--preset <name>] [--file <scenario.json>] [--name <n>]
  arm list-scenarios
  arm show-scenario <id>
  arm update-scenario <id> --file <scenario.json>
  arm fk <scenarioId> <theta1Deg> <theta2Deg>
  arm ik <scenarioId> <x> <y>
  arm configs <scenarioId>
  arm plan <scenarioId> [--seed <n>] [--step <r>] [--resolution <r>]
                     [--iterations <n>] [--goal-bias <p>] [--version v1]
                     [--idempotency-key <k>]
  arm list-plans [scenarioId]
  arm verify <planId>
  arm export <planId> [--csv] [--out <file>]
  arm compare <scenarioId> --seed-a <n> --seed-b <n>
  arm serve [--port <n>]        # binds 127.0.0.1, auto port when omitted
  arm stop
  arm demo

Presets: ${PRESETS.join(", ")}
`);
}

function argValue(args: string[], flag: string): string | undefined {
  const i = args.indexOf(flag);
  return i >= 0 ? args[i + 1] : undefined;
}

async function main(): Promise<void> {
  const [cmd, ...rest] = process.argv.slice(2);
  if (!cmd || cmd === "-h" || cmd === "--help" || cmd === "help") {
    usage();
    return;
  }

  switch (cmd) {
    case "init-scenario": {
      const preset = argValue(rest, "--preset") as PresetName | undefined;
      const file = argValue(rest, "--file");
      const overrideName = argValue(rest, "--name");
      let spec;
      if (file) {
        spec = JSON.parse(readFileSync(file, "utf-8"));
      } else if (preset) {
        if (!PRESETS.includes(preset)) throw new Error(`unknown preset ${preset}`);
        spec = generateScenario(preset);
      } else {
        spec = generateScenario("open");
      }
      if (overrideName) spec.name = overrideName;
      const { db, service } = getDb();
      const before = service.scenarios.list(1, 0).total;
      const row = service.createScenario(spec.name, spec);
      const after = service.scenarios.list(1, 0).total;
      console.log(`scenarios before=${before} after=${after}`);
      console.log(`created scenario id=${row.id} name=${row.name}`);
      console.log(JSON.stringify(spec, null, 2));
      db.close();
      return;
    }
    case "list-scenarios": {
      const { db, service } = getDb();
      const page = service.scenarios.list(200, 0);
      for (const r of page.items) {
        console.log(`${r.id}  v${r.spec_version}  ${r.name}  updated=${r.updated_at}`);
      }
      console.log(`total=${page.total}`);
      db.close();
      return;
    }
    case "show-scenario": {
      const { db, service } = getDb();
      const row = service.scenarios.get(rest[0]!);
      if (!row) throw new Error("not found");
      console.log(JSON.stringify(JSON.parse(row.spec), null, 2));
      db.close();
      return;
    }
    case "update-scenario": {
      const { db, service } = getDb();
      const id = rest[0]!;
      const file = argValue(rest, "--file");
      if (!file) throw new Error("--file required");
      const spec = JSON.parse(readFileSync(file, "utf-8"));
      const beforeRow = service.scenarios.get(id);
      if (!beforeRow) throw new Error("scenario not found");
      const plansBefore = service.plans.list(id, 200, 0).items.filter((p) => p.valid).length;
      const updated = service.updateScenario(id, { spec })!;
      const plansAfter = service.plans.list(id, 200, 0).items.filter((p) => p.valid).length;
      console.log(`spec v${beforeRow.spec_version} -> v${updated.spec_version}`);
      console.log(`valid plans before=${plansBefore} after=${plansAfter} (older plans marked invalid)`);
      db.close();
      return;
    }
    case "fk": {
      const [id, t1deg, t2deg] = rest;
      const { db, service } = getDb();
      const r = service.fk(id!, Number(t1deg) * Math.PI / 180, Number(t2deg) * Math.PI / 180);
      console.log(`elbow=(${fmt(r.joint.x)}, ${fmt(r.joint.y)}) tip=(${fmt(r.tip.x)}, ${fmt(r.tip.y)}) phi=${fmt(radToDeg(r.phi))}deg`);
      db.close();
      return;
    }
    case "ik": {
      const [id, xs, ys] = rest;
      const { db, service } = getDb();
      const sols = service.ik(id!, Number(xs), Number(ys));
      for (const s of sols) {
        console.log(`elbow=${s.elbow} theta1=${fmt(radToDeg(s.theta1))}deg theta2=${fmt(radToDeg(s.theta2))}deg inLimits=${s.inLimits} tip=(${fmt(s.fk.tip.x)},${fmt(s.fk.tip.y)})`);
      }
      db.close();
      return;
    }
    case "configs": {
      const { db, service } = getDb();
      console.log(JSON.stringify(service.solveStartGoal(rest[0]!), null, 2));
      db.close();
      return;
    }
    case "plan": {
      const id = rest[0]!;
      const params = {
        ...defaultParams(Number(argValue(rest, "--seed") ?? 12345)),
        ...(argValue(rest, "--step") ? { step: Number(argValue(rest, "--step")) } : {}),
        ...(argValue(rest, "--resolution") ? { resolution: Number(argValue(rest, "--resolution")) } : {}),
        ...(argValue(rest, "--iterations") ? { maxIterations: Number(argValue(rest, "--iterations")) } : {}),
        ...(argValue(rest, "--goal-bias") ? { goalBias: Number(argValue(rest, "--goal-bias")) } : {}),
        ...(argValue(rest, "--version") ? { version: String(argValue(rest, "--version")) } : {}),
      };
      const { db, service } = getDb();
      const { plan, reused } = service.plan(id, params, argValue(rest, "--idempotency-key"));
      console.log(`planId=${plan.id} reused=${reused} status=${plan.report.status} valid=${plan.valid}`);
      console.log(`reason=${plan.report.reason ?? "-"} iterations=${plan.report.iterations} nodes=${plan.report.nodes} segments=${plan.report.segments} collisions=${plan.report.collisions} elapsedMs=${plan.report.elapsedMs}`);
      console.log(`start=(${fmt(plan.configStart.theta1)},${fmt(plan.configStart.theta2)}) goal=(${fmt(plan.configGoal.theta1)},${fmt(plan.configGoal.theta2)}) pathLength=${plan.path.length}`);
      db.close();
      return;
    }
    case "list-plans": {
      const { db, service } = getDb();
      const page = service.plans.list(rest[0] || undefined, 200, 0);
      for (const p of page.items) {
        const report = JSON.parse(p.report) as { status: string; reason?: string };
        console.log(`${p.id}  scenario=${p.scenario_id}  status=${report.status}  valid=${p.valid === 1}  ${report.reason ?? ""}  created=${p.created_at}`);
      }
      console.log(`total=${page.total}`);
      db.close();
      return;
    }
    case "verify": {
      const { db, service } = getDb();
      const v = service.verify(rest[0]!);
      console.log(`storedValid=${v.storedValid} currentSpec=v${v.specVersion}`);
      console.log(`edgeCheck ok=${v.validation.ok} edges=${v.validation.edges} samples=${v.validation.totalSamples}`);
      if (v.validation.firstFailure) {
        console.log(`firstFailure edge=${v.validation.firstFailure.edgeIndex} t=${fmt(v.validation.firstFailure.t)} obstacle=${v.validation.firstFailure.obstacle}`);
      }
      console.log(`startCollision=${v.startHit} goalCollision=${v.goalHit}`);
      db.close();
      return;
    }
    case "export": {
      const { db, service } = getDb();
      const csv = argValue(rest, "--csv") !== undefined;
      const out = argValue(rest, "--out");
      const data = csv ? (service.trajectory(rest[0]!, "csv") as string) : JSON.stringify(service.trajectory(rest[0]!, "json"), null, 2);
      if (out) {
        mkdirSync(join(process.cwd(), "tmp"), { recursive: true });
        const target = isAbsolute(out) ? out : join(process.cwd(), out);
        writeFileSync(target, data);
        console.log(`wrote ${target}`);
      } else {
        console.log(data);
      }
      db.close();
      return;
    }
    case "compare": {
      const id = rest[0]!;
      const { db, service } = getDb();
      const a = defaultParams(Number(argValue(rest, "--seed-a") ?? 1));
      const b = defaultParams(Number(argValue(rest, "--seed-b") ?? 2));
      console.log(JSON.stringify(service.compare(id, a, b), null, 2));
      db.close();
      return;
    }
    case "serve": {
      mkdirSync(RUN_DIR, { recursive: true });
      const port = Number(argValue(rest, "--port") ?? 0);
      const started = await startApi(DB_PATH, port);
      writeFileSync(PID_FILE, String(process.pid));
      writeFileSync(PORT_FILE, String(started.port));
      console.log(`API listening on http://127.0.0.1:${started.port} (pid ${process.pid})`);
      console.log(`pid file: ${PID_FILE}`);
      const shutdown = () => {
        started.close().then(() => process.exit(0));
      };
      process.on("SIGINT", shutdown);
      process.on("SIGTERM", shutdown);
      return;
    }
    case "stop": {
      if (!existsSync(PID_FILE)) {
        console.log("no pid file; server not running (managed by this project only)");
        return;
      }
      const pid = Number(readFileSync(PID_FILE, "utf-8").trim());
      try {
        process.kill(pid, "SIGTERM");
        console.log(`sent SIGTERM to pid ${pid}`);
      } catch {
        console.log(`pid ${pid} not running; removing stale pid file`);
      }
      return;
    }
    case "demo": {
      const { runDemo } = await import("../demo/demo.js");
      await runDemo();
      return;
    }
    default:
      usage();
      process.exitCode = 1;
  }
}

main().catch((e) => {
  console.error(`error: ${e.message}`);
  process.exit(1);
});
