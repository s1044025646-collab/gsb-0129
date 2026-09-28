import { createServer, type IncomingMessage, type Server, type ServerResponse } from "node:http";
import { openDatabase, parsePaging } from "../db/database.js";
import { validateParams, validateScenario } from "../core/validation.js";
import { PlanningService, ServiceError, parsePlan, parseScenario } from "../service/planningService.js";
import { defaultParams } from "../core/validation.js";
import { generateScenario, PRESETS } from "../core/generator.js";

export interface ApiContext {
  service: PlanningService;
}

export interface StartedServer {
  server: Server;
  port: number;
  close(): Promise<void>;
}

const JSON_HEADERS = { "content-type": "application/json; charset=utf-8" };

function sendJson(res: ServerResponse, status: number, body: unknown): void {
  res.writeHead(status, JSON_HEADERS);
  res.end(JSON.stringify(body));
}

function sendError(res: ServerResponse, status: number, code: string, message: string, details?: unknown): void {
  sendJson(res, status, { error: { code, message, details } });
}

async function readJson(req: IncomingMessage): Promise<unknown> {
  const chunks: Buffer[] = [];
  for await (const c of req) chunks.push(c as Buffer);
  const raw = Buffer.concat(chunks).toString("utf-8");
  if (!raw) return {};
  try {
    return JSON.parse(raw);
  } catch {
    throw new HttpError(400, "INVALID_JSON", "request body was not valid JSON");
  }
}

class HttpError extends Error {
  constructor(public status: number, public code: string, message: string, public details?: unknown) {
    super(message);
  }
}

export function createApp(ctx: ApiContext) {
  const { service } = ctx;

  const handler = async (req: IncomingMessage, res: ServerResponse): Promise<void> => {
    try {
      const url = new URL(req.url ?? "/", "http://127.0.0.1");
      const path = url.pathname.replace(/\/+$/, "") || "/";
      const method = req.method ?? "GET";

      if (method === "GET" && path === "/health") {
        sendJson(res, 200, { ok: true });
        return;
      }

      if (method === "GET" && path === "/presets") {
        sendJson(res, 200, { presets: PRESETS });
        return;
      }

      if (method === "POST" && path === "/presets/generate") {
        const body = (await readJson(req)) as { name?: string };
        if (!body.name || !PRESETS.includes(body.name as never)) {
          throw new HttpError(400, "VALIDATION_ERROR", "unknown preset", { allowed: PRESETS });
        }
        const spec = generateScenario(body.name as never);
        const row = service.createScenario(spec.name, spec);
        sendJson(res, 201, { id: row.id, spec: JSON.parse(row.spec) });
        return;
      }

      if (method === "POST" && path === "/scenarios") {
        const body = await readJson(req);
        const v = validateScenario(body);
        if (!v.ok) throw new HttpError(400, "VALIDATION_ERROR", "invalid scenario", v.errors);
        const row = service.createScenario(v.value.name, v.value);
        sendJson(res, 201, { id: row.id, specVersion: row.spec_version, spec: v.value });
        return;
      }

      if (method === "GET" && path === "/scenarios") {
        const { limit, offset } = parsePaging({
          limit: url.searchParams.get("limit") ?? undefined,
          offset: url.searchParams.get("offset") ?? undefined,
        });
        const page = service.scenarios.list(limit, offset);
        sendJson(res, 200, {
          items: page.items.map((r) => ({
            id: r.id,
            name: r.name,
            specVersion: r.spec_version,
            updatedAt: r.updated_at,
          })),
          total: page.total,
          limit: page.limit,
          offset: page.offset,
        });
        return;
      }

      const scenarioMatch = path.match(/^\/scenarios\/([^/]+)$/);
      if (scenarioMatch) {
        const id = scenarioMatch[1]!;
        const row = service.scenarios.get(id);
        if (!row) throw new HttpError(404, "NOT_FOUND", "scenario not found");
        if (method === "GET") {
          sendJson(res, 200, {
            id: row.id,
            name: row.name,
            specVersion: row.spec_version,
            spec: parseScenario(row),
            createdAt: row.created_at,
            updatedAt: row.updated_at,
          });
          return;
        }
        if (method === "PUT") {
          const body = (await readJson(req)) as { name?: unknown; spec?: unknown };
          const patch: { name?: string; spec?: never } = {};
          if (body.name !== undefined) {
            if (typeof body.name !== "string" || !body.name.trim()) {
              throw new HttpError(400, "VALIDATION_ERROR", "name must be non-empty string");
            }
            patch.name = body.name;
          }
          if (body.spec !== undefined) {
            const v = validateScenario(body.spec);
            if (!v.ok) throw new HttpError(400, "VALIDATION_ERROR", "invalid scenario", v.errors);
            (patch as { spec?: unknown }).spec = v.value;
          }
          const updated = service.updateScenario(id, patch);
          if (!updated) throw new HttpError(404, "NOT_FOUND", "scenario not found");
          sendJson(res, 200, {
            id: updated.id,
            specVersion: updated.spec_version,
            plansInvalidated: true,
          });
          return;
        }
      }

      const ikMatch = path.match(/^\/scenarios\/([^/]+)\/ik$/);
      if (ikMatch && method === "GET") {
        const id = ikMatch[1]!;
        const x = Number(url.searchParams.get("x"));
        const y = Number(url.searchParams.get("y"));
        if (!Number.isFinite(x) || !Number.isFinite(y)) {
          throw new HttpError(400, "VALIDATION_ERROR", "x and y must be finite numbers");
        }
        const sols = service.ik(id, x, y);
        sendJson(res, 200, { solutions: sols });
        return;
      }

      const fkMatch = path.match(/^\/scenarios\/([^/]+)\/fk$/);
      if (fkMatch && method === "GET") {
        const id = fkMatch[1]!;
        const theta1 = Number(url.searchParams.get("theta1"));
        const theta2 = Number(url.searchParams.get("theta2"));
        if (!Number.isFinite(theta1) || !Number.isFinite(theta2)) {
          throw new HttpError(400, "VALIDATION_ERROR", "theta1 and theta2 must be finite numbers");
        }
        sendJson(res, 200, service.fk(id, theta1, theta2));
        return;
      }

      const configsMatch = path.match(/^\/scenarios\/([^/]+)\/configs$/);
      if (configsMatch && method === "GET") {
        sendJson(res, 200, service.solveStartGoal(configsMatch[1]!));
        return;
      }

      const planMatch = path.match(/^\/scenarios\/([^/]+)\/plans$/);
      if (planMatch) {
        const id = planMatch[1]!;
        if (method === "POST") {
          const body = (await readJson(req)) as { params?: unknown };
          const pv = validateParams(body.params ?? defaultParams());
          if (!pv.ok) throw new HttpError(400, "VALIDATION_ERROR", "invalid params", pv.errors);
          const key = req.headers["idempotency-key"];
          const idem = Array.isArray(key) ? key[0] : key;
          const { plan, reused } = service.plan(id, pv.value, idem ?? undefined);
          sendJson(res, reused ? 200 : 201, { plan, reused });
          return;
        }
        if (method === "GET") {
          const { limit, offset } = parsePaging({
            limit: url.searchParams.get("limit") ?? undefined,
            offset: url.searchParams.get("offset") ?? undefined,
          });
          const page = service.plans.list(id, limit, offset);
          sendJson(res, 200, {
            items: page.items.map(parsePlan),
            total: page.total,
            limit: page.limit,
            offset: page.offset,
          });
          return;
        }
      }

      if (method === "GET" && path === "/plans") {
        const scenarioId = url.searchParams.get("scenarioId") ?? undefined;
        const { limit, offset } = parsePaging({
          limit: url.searchParams.get("limit") ?? undefined,
          offset: url.searchParams.get("offset") ?? undefined,
        });
        const page = service.plans.list(scenarioId, limit, offset);
        sendJson(res, 200, {
          items: page.items.map(parsePlan),
          total: page.total,
          limit: page.limit,
          offset: page.offset,
        });
        return;
      }

      const planIdMatch = path.match(/^\/plans\/([^/]+)(\/(verify|trajectory))?$/);
      if (planIdMatch && method === "GET") {
        const id = planIdMatch[1]!;
        const sub = planIdMatch[3];
        if (sub === "verify") {
          const resolution = url.searchParams.get("resolution");
          sendJson(res, 200, service.verify(id, resolution ? Number(resolution) : undefined));
          return;
        }
        if (sub === "trajectory") {
          const format = url.searchParams.get("format") === "csv" ? "csv" : "json";
          if (format === "csv") {
            const csv = service.trajectory(id, "csv") as string;
            res.writeHead(200, { "content-type": "text/csv; charset=utf-8" });
            res.end(csv);
            return;
          }
          sendJson(res, 200, { rows: service.trajectory(id, "json") });
          return;
        }
        const row = service.plans.get(id);
        if (!row) throw new HttpError(404, "NOT_FOUND", "plan not found");
        sendJson(res, 200, parsePlan(row));
        return;
      }

      if (method === "POST" && path === "/compare") {
        const body = (await readJson(req)) as { scenarioId?: unknown; paramsA?: unknown; paramsB?: unknown };
        if (typeof body.scenarioId !== "string") {
          throw new HttpError(400, "VALIDATION_ERROR", "scenarioId required");
        }
        const va = validateParams(body.paramsA ?? defaultParams());
        const vb = validateParams(body.paramsB ?? defaultParams());
        if (!va.ok || !vb.ok) {
          throw new HttpError(400, "VALIDATION_ERROR", "invalid params", { a: va.ok ? undefined : va.errors, b: vb.ok ? undefined : vb.errors });
        }
        sendJson(res, 200, service.compare(body.scenarioId, va.value, vb.value));
        return;
      }

      sendError(res, 404, "NOT_FOUND", `no route for ${method} ${path}`);
    } catch (e) {
      if (e instanceof HttpError) {
        sendError(res, e.status, e.code, e.message, e.details);
        return;
      }
      if (e instanceof ServiceError) {
        const status = e.code === "NOT_FOUND" ? 404 : 422;
        sendError(res, status, e.code, e.message);
        return;
      }
      sendError(res, 500, "INTERNAL", (e as Error).message);
    }
  };

  return handler;
}

/**
 * Start the API. Always binds 127.0.0.1 (never 0.0.0.0) and uses port 0
 * (OS-assigned free port) unless an explicit port is supplied.
 */
export function startApi(dbPath: string, port = 0, host = "127.0.0.1"): Promise<StartedServer> {
  const db = openDatabase(dbPath);
  const service = new PlanningService(db);
  const server = createServer(createApp({ service }));
  return new Promise((resolve, reject) => {
    server.once("error", reject);
    server.listen(port, host, () => {
      const addr = server.address();
      const actualPort = typeof addr === "object" && addr ? addr.port : port;
      resolve({
        server,
        port: actualPort,
        close: () =>
          new Promise<void>((res, rej) => {
            server.close((err) => (err ? rej(err) : res()));
            db.close();
          }),
      });
    });
  });
}
