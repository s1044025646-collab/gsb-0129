import { createServer, Server } from "node:http";
import { AddressInfo } from "node:net";
import { openDatabase } from "./db.js";
import { Store } from "./store.js";
import { KinematicsService } from "./service.js";
import {
  ValidationError,
  validateArm,
  validateJoints,
  validateObstacles,
  validatePagination,
  validateParams,
  validatePose,
} from "./validation.js";
import { IKError } from "./kinematics.js";
import { DEFAULT_PARAMS } from "./planner.js";

export interface AppContext {
  store: Store;
  service: KinematicsService;
}

export function createApp(dbPath: string): { server: Server; context: AppContext } {
  const db = openDatabase(dbPath);
  const store = new Store(db);
  const service = new KinematicsService(store);
  const context: AppContext = { store, service };

  const server = createServer((req, res) => {
    handle(req, res, context).catch((err) => {
      sendError(res, err);
    });
  });
  return { server, context };
}

async function readJson(req: import("node:http").IncomingMessage): Promise<unknown> {
  const chunks: Buffer[] = [];
  for await (const chunk of req) chunks.push(chunk as Buffer);
  const raw = Buffer.concat(chunks).toString("utf8");
  if (!raw) return {};
  try {
    return JSON.parse(raw);
  } catch {
    throw new ValidationError([{ field: "body", message: "invalid JSON" }]);
  }
}

function sendJson(res: import("node:http").ServerResponse, status: number, body: unknown): void {
  const payload = JSON.stringify(body);
  res.writeHead(status, {
    "content-type": "application/json",
    "content-length": Buffer.byteLength(payload),
  });
  res.end(payload);
}

function sendError(res: import("node:http").ServerResponse, err: unknown): void {
  if (err instanceof ValidationError) {
    sendJson(res, 400, {
      error: { code: "VALIDATION_ERROR", message: err.message, details: err.details },
    });
    return;
  }
  if (err instanceof IKError) {
    sendJson(res, 422, { error: { code: err.code, message: err.message } });
    return;
  }
  if (err instanceof Error && err.message === "SCENARIO_NOT_FOUND") {
    sendJson(res, 404, { error: { code: "SCENARIO_NOT_FOUND", message: "Scenario not found" } });
    return;
  }
  const message = err instanceof Error ? err.message : "Internal error";
  sendJson(res, 500, { error: { code: "INTERNAL", message } });
}

async function handle(
  req: import("node:http").IncomingMessage,
  res: import("node:http").ServerResponse,
  ctx: AppContext
): Promise<void> {
  const url = new URL(req.url ?? "/", "http://127.0.0.1");
  const path = url.pathname.replace(/\/+$/, "") || "/";
  const method = req.method ?? "GET";

  // Health
  if (method === "GET" && path === "/health") {
    sendJson(res, 200, { ok: true });
    return;
  }

  // Scenarios
  if (method === "GET" && path === "/api/scenarios") {
    const { limit, offset } = validatePagination(url.searchParams);
    sendJson(res, 200, ctx.store.listScenarios(limit, offset));
    return;
  }
  if (method === "POST" && path === "/api/scenarios") {
    const body = (await readJson(req)) as Record<string, unknown>;
    const record = ctx.store.upsertScenario({
      id: typeof body.id === "string" ? body.id : undefined,
      name: typeof body.name === "string" ? body.name : "unnamed",
      description: typeof body.description === "string" ? body.description : "",
      arm: validateArm(body.arm),
      obstacles: validateObstacles(body.obstacles ?? []),
      start: validateJoints(body.start, "start"),
      goal: validateJoints(body.goal, "goal"),
      goalPose: validatePose(body.goalPose ?? { x: 0, y: 0, phi: null }),
    });
    sendJson(res, 201, record);
    return;
  }

  const scenarioMatch = /^\/api\/scenarios\/([^/]+)$/.exec(path);
  if (method === "GET" && scenarioMatch) {
    const record = ctx.store.getScenario(scenarioMatch[1]!);
    if (!record) {
      sendJson(res, 404, { error: { code: "NOT_FOUND", message: "Scenario not found" } });
      return;
    }
    sendJson(res, 200, record);
    return;
  }

  // Kinematics stateless compute
  if (method === "POST" && path === "/api/fk") {
    const body = (await readJson(req)) as Record<string, unknown>;
    const arm = validateArm(body.arm);
    const q = validateJoints(body.joints, "joints");
    sendJson(res, 200, ctx.service.fk(arm, q));
    return;
  }
  if (method === "POST" && path === "/api/ik") {
    const body = (await readJson(req)) as Record<string, unknown>;
    const arm = validateArm(body.arm);
    const pose = validatePose(body.pose);
    sendJson(res, 200, ctx.service.ik(arm, pose));
    return;
  }

  // Plans
  if (method === "GET" && path === "/api/plans") {
    const { limit, offset } = validatePagination(url.searchParams);
    const scenarioId = url.searchParams.get("scenarioId");
    sendJson(res, 200, ctx.store.listPlans(scenarioId, limit, offset));
    return;
  }

  const planMatch = /^\/api\/plans\/([^/]+)$/.exec(path);
  if (method === "GET" && planMatch) {
    const plan = ctx.store.getPlan(planMatch[1]!);
    if (!plan) {
      sendJson(res, 404, { error: { code: "NOT_FOUND", message: "Plan not found" } });
      return;
    }
    sendJson(res, 200, ctx.store.refreshPlanValidity(plan.id));
    return;
  }
  if (method === "POST" && path === "/api/plans") {
    const body = (await readJson(req)) as Record<string, unknown>;
    const scenario = ctx.store.getScenario(String(body.scenarioId ?? ""));
    if (!scenario) {
      sendJson(res, 404, { error: { code: "SCENARIO_NOT_FOUND", message: "Scenario not found" } });
      return;
    }
    const params = body.params
      ? validateParams(body.params as Partial<typeof DEFAULT_PARAMS>)
      : { ...DEFAULT_PARAMS };
    const clientRequestId =
      typeof body.clientRequestId === "string" ? body.clientRequestId : undefined;
    const { record, reused } = ctx.service.plan(scenario, params, clientRequestId);
    sendJson(res, reused ? 200 : 201, { reused, plan: record });
    return;
  }

  const verifyMatch = /^\/api\/plans\/([^/]+)\/verify$/.exec(path);
  if (method === "POST" && verifyMatch) {
    const plan = ctx.store.getPlan(verifyMatch[1]!);
    if (!plan) {
      sendJson(res, 404, { error: { code: "NOT_FOUND", message: "Plan not found" } });
      return;
    }
    sendJson(res, 200, { planId: plan.id, verification: ctx.service.verifyPlan(plan) });
    return;
  }

  const exportMatch = /^\/api\/plans\/([^/]+)\/export$/.exec(path);
  if (method === "GET" && exportMatch) {
    const plan = ctx.store.getPlan(exportMatch[1]!);
    if (!plan) {
      sendJson(res, 404, { error: { code: "NOT_FOUND", message: "Plan not found" } });
      return;
    }
    sendJson(res, 200, { planId: plan.id, ...ctx.service.exportTrajectory(plan) });
    return;
  }

  if (method === "POST" && path === "/api/compare") {
    const body = (await readJson(req)) as Record<string, unknown>;
    const scenario = ctx.store.getScenario(String(body.scenarioId ?? ""));
    if (!scenario) {
      sendJson(res, 404, { error: { code: "SCENARIO_NOT_FOUND", message: "Scenario not found" } });
      return;
    }
    const a = validateParams((body.paramsA ?? {}) as Partial<typeof DEFAULT_PARAMS>);
    const b = validateParams((body.paramsB ?? {}) as Partial<typeof DEFAULT_PARAMS>);
    sendJson(res, 200, ctx.service.compareParams(scenario, a, b));
    return;
  }

  sendJson(res, 404, { error: { code: "NOT_FOUND", message: `No route for ${method} ${path}` } });
}

export function startServer(
  dbPath: string,
  preferredPort = 0
): Promise<{ server: Server; port: number; context: AppContext }> {
  const { server, context } = createApp(dbPath);
  return new Promise((resolve) => {
    // Bind loopback only; port 0 lets the OS assign an ephemeral free port.
    server.listen(preferredPort, "127.0.0.1", () => {
      const port = (server.address() as AddressInfo).port;
      resolve({ server, port, context });
    });
  });
}
