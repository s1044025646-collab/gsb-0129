# Planar 2-DOF Arm Kinematics, Collision & Path-Planning Backend

Windows-local, offline backend for a fictional **two-revolute-joint, two-rigid-link**
planar manipulator. It provides only an HTTP API, a CLI, a fictional scenario
generator and automated tests — **there is no web UI and no real-robot control**.

The stack is TypeScript + Node.js and the built-in synchronous SQLite module
(`node:sqlite`), so there are no native modules to compile.

## Capabilities

- Self-implemented forward and inverse kinematics for a 2R arm.
- IK enumerates both **elbow-up / elbow-down** branches, keeps every periodic
  representative satisfying the joint limits, and reports:
  - `UNREACHABLE_TOO_FAR` / `UNREACHABLE_TOO_NEAR`
  - singular fully-extended / fully-folded poses
  - `JOINT_LIMIT_VIOLATION` per rejected branch
  - the +/-PI angle periodic boundary (shortest signed interpolation)
- Collision checking is against **whole link segments** (base, link1, link2)
  and every circular obstacle, not only the end-effector point.
- Joint-space motion verification uses a fixed joint step plus **adaptive
  cartesian subdivision**, so thin obstacles cannot be tunnelled through.
- Planning is a seeded, deterministic **bidirectional RRT-Connect** with an
  explicit iteration budget. Exhausting the budget returns `NOT_FOUND` ("search
  budget exhausted") — it never claims geometric impossibility and never emits a
  colliding path. Fixed seeds reproduce identical trajectories.
- Stored plans keep an **input snapshot**; changing arm geometry or obstacles
  changes the scenario hash and marks old plans `valid=false / SCENARIO_CHANGED`.
- Repeated submissions with the same `clientRequestId` are **idempotent**.
- API has request validation, explicit error codes and paginated list queries.

## Requirements

- Node.js >= 22 with the built-in `node:sqlite` module (developed on Node 25).
  If a build of Node omits it, use Node 25.
- npm (ships with Node).
- Windows PowerShell is used in the commands below; they also work in any shell.

All data, databases, exported trajectories, logs and temporary files stay under
`./data` inside this project.

## Install

```powershell
npm install
```

## Test

```powershell
npm test
```

Runs the built-in `node:test` suite (33 tests) covering:

- double IK solutions, full extension/fold, joint limits, periodic wrapping
- FK <-> IK round trips and degree/radian conversion
- whole-link collision where the tip is safe, collision boundary, thin-obstacle
  tunnelling via subdivision
- path continuity, detour routing, narrow passage, start-in-collision
- fixed-seed/version reproduction
- transaction rollback, persistence after restart, pagination, idempotency,
  invalidation after geometry change, API validation/error codes

Type-check without emitting:

```powershell
npx tsc -p tsconfig.json
```

## Repeatable end-to-end demo

```powershell
npm run demo
```

Uses a frozen clock and fixed seeds. It creates all fixture scenarios, plans and
re-verifies them, replays an idempotent submission, invalidates a plan after an
obstacle is added, and demonstrates budget exhaustion. Output DB: `data/demo.sqlite`.

## Start the API

```powershell
npm start
```

The server binds **loopback only** (`127.0.0.1`) and, by default, asks the OS for
an ephemeral free port (`0`), which is printed at startup:

```
API listening on http://127.0.0.1:54902 (loopback only)
```

To request a specific loopback port:

```powershell
node --import tsx src/cli.ts serve --port 8080
```

### Stop

If started in the foreground, press `Ctrl+C`. If started detached, stop only the
process you launched for this project, e.g. capture the PID:

```powershell
$p = Start-Process node -ArgumentList "--import","tsx","src/cli.ts","serve" -PassThru -WindowStyle Hidden
Stop-Process $p.Id -Force
```

The project starts and stops only its own Node process; it never touches other
programs' processes.

## CLI

```powershell
node --import tsx src/cli.ts help
```

Common flows:

```powershell
node --import tsx src/cli.ts create-scenario --kind detour --id detour
node --import tsx src/cli.ts list-scenarios
node --import tsx src/cli.ts fk --scenario detour --theta1-deg 20 --theta2-deg 40
node --import tsx src/cli.ts ik --scenario detour
node --import tsx src/cli.ts plan --scenario detour --seed 7 --idempotency-key k1
node --import tsx src/cli.ts verify --scenario detour
node --import tsx src/cli.ts compare --scenario detour --seed-a 1 --seed-b 2
node --import tsx src/cli.ts export --scenario detour --out data/traj.json
node --import tsx src/cli.ts list-plans --scenario detour
```

Fixture scenario kinds: `clear`, `tipSafeLinkHits`, `detour`,
`startCollision`, `narrowPassage`. `create-scenario --seed N` produces a
procedural fictional scene (fixed seed => fixed scene).

CLI planning/verification commands print a `BEFORE`/`AFTER` state summary.

## HTTP API

All request/response bodies are JSON.

| Method | Path | Purpose |
| --- | --- | --- |
| GET | `/health` | Liveness |
| POST | `/api/scenarios` | Create/update a scenario (fixed `id` upserts) |
| GET | `/api/scenarios?limit=&offset=` | Paginated scenarios |
| GET | `/api/scenarios/:id` | One scenario |
| POST | `/api/fk` | Stateless forward kinematics |
| POST | `/api/ik` | Stateless inverse kinematics |
| POST | `/api/plans` | Plan for a scenario; optional `clientRequestId`, `params` |
| GET | `/api/plans?scenarioId=&limit=&offset=` | Paginated plans |
| GET | `/api/plans/:id` | Plan (validity refreshed against current hash) |
| POST | `/api/plans/:id/verify` | Independent per-segment collision verification |
| GET | `/api/plans/:id/export` | Joint-angle and cartesian tip trajectory |
| POST | `/api/compare` | Compare two parameter sets on one scenario |

Errors use HTTP status plus a stable `error.code`, e.g. `VALIDATION_ERROR`
(400, with field details), `UNREACHABLE_TOO_FAR` (422), `SCENARIO_NOT_FOUND`
(404), `NOT_FOUND` (404), `INTERNAL` (500).

Example:

```powershell
$body = @{
  scenarioId = "detour"
  clientRequestId = "run-001"
  params = @{ seed = 7; maxIterations = 4000; step = 0.25; paramsVersion = "v1" }
} | ConvertTo-Json
Invoke-RestMethod -Method Post -Uri http://127.0.0.1:<port>/api/plans -Body $body -ContentType application/json
```

## Project layout

- `src/geometry.ts` — vectors, angles/periodicity, segment-circle distance
- `src/kinematics.ts` — arm spec, FK/IK, limits, periodic wrapping, interpolation
- `src/collision.ts` — pose and adaptively-subdivided motion collision checks
- `src/planner.ts` — seeded bidirectional RRT-Connect, budget, pruning, densify
- `src/rng.ts` — deterministic PRNG
- `src/scenario.ts` — fictional fixtures and seeded procedural generator
- `src/db.ts`, `src/store.ts` — repeatable migration, persistence, hashing, idempotency
- `src/service.ts` — plan / verify / compare / export operations
- `src/server.ts` — loopback HTTP API with validation and error codes
- `src/cli.ts`, `src/demo.ts` — command line and repeatable demo
- `test/` — automated tests

## Migrations

Schema creation is idempotent (`CREATE TABLE IF NOT EXISTS`) and runs on every
startup, so restarting or re-running is safe.
