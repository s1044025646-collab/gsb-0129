# planar-arm-planner

Windows-local backend for kinematics, collision checking and joint-space path
planning of a **2-DOF planar manipulator** (two revolute joints, two rigid
links). It exposes an HTTP API, a CLI, a fictional scenario generator, an
end-to-end demo and automated tests. There is **no web UI and no real-robot
control** anywhere in the project.

All data, databases and temporary files stay inside this project directory
(`data/`, `tmp/`). The API binds `127.0.0.1` only and picks an OS-assigned
free port by default.

## Requirements

- Node.js >= 22 (developed on Node 25) — uses the built-in `node:sqlite`, so
  **there are zero runtime dependencies**.
- npm (ships with Node.js)
- Windows PowerShell is used in the commands below; the CLI itself is
  cross-platform.

## Install

```powershell
npm install
```

## Test

```powershell
npm test
```

The suite (28 tests) covers:

- dual elbow-up / elbow-down inverse solutions and FK round-trip
- fully stretched and fully folded singularities
- unreachable targets (outer reach / inner workspace) with error codes
- joint-limit filtering and periodic angle boundaries
- rad/deg conversion and angle wrapping
- whole-link collisions, including "tip safe but link colliding"
- tangency boundary and dense edge sampling that catches tunneling through a
  thin obstacle
- path continuity, fixed-seed version reproduction, detour / narrow-passage
  plans, start-in-collision and exhausted-budget failure contracts
- idempotent submission, plan invalidation on geometry change, transaction
  rollback, idempotent/repeatable migrations, restart persistence and
  pagination

Type checking:

```powershell
npx tsc -p tsconfig.json
```

## Repeatable end-to-end demo

```powershell
npm run demo
```

The demo uses an injected clock and fixed RNG seeds, so output is identical
on every run. It plans all five fictional presets, checks idempotent
resubmission, verifies each successful path, and demonstrates that editing a
scenario's obstacles marks the old plan invalid and fails re-verification.
Artifacts go to `tmp/demo/`.

## Start the API

```powershell
npm start
```

Equivalent to `npx tsx src/cli/cli.ts serve`. On start it prints e.g.:

```
API listening on http://127.0.0.1:56840 (pid 46840)
pid file: E:\...\tmp\server.pid
```

- Binds **127.0.0.1 only**; port `0` means the OS chooses a free port, which
  is written to `tmp/server.port`.
- To pin a port: `npx tsx src/cli/cli.ts serve --port 8787`.

### Stop the API

```powershell
npx tsx src/cli/cli.ts stop
```

Start/stop only manages the process whose pid is recorded in this project's
`tmp/server.pid`; it never touches unrelated processes. Ctrl+C in the server
window also works.

## CLI

Run with `npx tsx src/cli/cli.ts <command>` (or `npm run cli -- <command>`).

```powershell
# Fictional scenarios
npx tsx src/cli/cli.ts init-scenario --preset narrow
npx tsx src/cli/cli.ts init-scenario --file my-scenario.json
npx tsx src/cli/cli.ts list-scenarios
npx tsx src/cli/cli.ts show-scenario <scenarioId>
npx tsx src/cli/cli.ts update-scenario <scenarioId> --file my-scenario.json

# Kinematics (angles on CLI are degrees)
npx tsx src/cli/cli.ts fk <scenarioId> 30 -20
npx tsx src/cli/cli.ts ik <scenarioId> 1.2 0.5
npx tsx src/cli/cli.ts configs <scenarioId>

# Planning / validation / export
npx tsx src/cli/cli.ts plan <scenarioId> --seed 7
npx tsx src/cli/cli.ts plan <scenarioId> --step 0.25 --resolution 0.02 `
  --iterations 4000 --goal-bias 0.12 --version v1 --idempotency-key k1
npx tsx src/cli/cli.ts list-plans [scenarioId]
npx tsx src/cli/cli.ts verify <planId>
npx tsx src/cli/cli.ts export <planId> --csv --out tmp/trajectory.csv
npx tsx src/cli/cli.ts compare <scenarioId> --seed-a 1 --seed-b 2
```

Mutating commands print before/after state (row counts, spec version, count
of plans invalidated).

## HTTP API

JSON in/out. Errors always look like `{"error":{"code","message","details"}}`.

| Method | Path | Purpose |
| --- | --- | --- |
| GET | `/health` | liveness |
| GET | `/presets` | list fictional scenario presets |
| POST | `/presets/generate` | persist a preset `{name}` |
| POST | `/scenarios` | validate + create scenario |
| GET | `/scenarios?limit=&offset=` | paginated scenarios |
| GET | `/scenarios/:id` | full scenario snapshot |
| PUT | `/scenarios/:id` | edit name/spec; old plans invalidated |
| GET | `/scenarios/:id/fk?theta1=&theta2=` | forward kinematics (radians) |
| GET | `/scenarios/:id/ik?x=&y=` | both elbow solutions |
| GET | `/scenarios/:id/configs` | start/goal IK + orientation error |
| POST | `/scenarios/:id/plans` | plan; `Idempotency-Key` header supported |
| GET | `/scenarios/:id/plans?limit=&offset=` | paginated plans for scenario |
| GET | `/plans?scenarioId=&limit=&offset=` | paginated plans globally |
| GET | `/plans/:id` | stored plan with input snapshot |
| GET | `/plans/:id/verify?resolution=` | re-verify every interpolated segment |
| GET | `/plans/:id/trajectory?format=json\|csv` | joint angles + elbow/tip trajectory |
| POST | `/compare` | run two parameter versions side by side |

Error codes include `VALIDATION_ERROR` (400), `NOT_FOUND` (404),
`UNREACHABLE_TOO_FAR`, `UNREACHABLE_TOO_NEAR`, `NO_ELBOW_IN_LIMITS`,
`START_IN_COLLISION`, `GOAL_IN_COLLISION`, `START_UNREACHABLE`,
`GOAL_UNREACHABLE` and `BUDGET_EXHAUSTED` (422).

Planning parameters (versioned):

```json
{
  "version": "v1",
  "step": 0.25,
  "resolution": 0.02,
  "maxIterations": 4000,
  "goalBias": 0.12,
  "seed": 7
}
```

`resolution` must be <= `step`: every RRT edge is densely subdivided and
whole-arm collision checked, so thin obstacles cannot be tunneled through.

## How planning works

- **Kinematics** are self-implemented: FK via link transforms; IK via the
  law of cosines, enumerating elbow-up/down. Fully stretched/folded poses are
  singular and return the single coincident solution. Angles use periodic
  equivalents so limits near +/-pi are handled without discontinuity.
- **Collision** tests both links (base-to-elbow and elbow-to-tip capsules)
  against every circular obstacle, not only the end-effector.
- **Planning** is a seeded RRT in joint space. A fixed seed reproduces the
  exact tree and path. Extension and goal-connection edges are both densely
  verified. If the iteration budget runs out the result is persisted as
  `no_path` / `BUDGET_EXHAUSTED` — it never claims geometric impossibility
  and never returns a colliding path.
- **Snapshots & invalidation**: each plan stores its parameters plus the
  chosen start/goal joint configurations. Editing a scenario's obstacles or
  dimensions bumps the spec version and marks its plans `valid=false` in the
  same transaction; `verify` re-checks the stored path against current
  geometry.
- **Idempotency**: repeat plan submissions carrying the same
  `Idempotency-Key` return the original stored plan and preserve its input
  snapshot.

## Storage

- `data/arm.db` — SQLite database for the CLI/API (WAL mode), created on
  first use.
- `tmp/` — pid/port files, demo and test databases and exported files.

Both directories are git-ignored and safe to delete while the server is
stopped. Migrations use `CREATE TABLE IF NOT EXISTS` plus a
`PRAGMA user_version` and can be executed repeatedly.

## Project layout

```
src/core/        geometry, kinematics, collision, seeded RNG, RRT planner,
                 trajectory export, fictional presets, input validation
src/db/          node:sqlite schema, migrations, repositories, paging
src/service/     transactional orchestration, plan verify and compare
src/api/         loopback-only HTTP server with explicit error codes
src/cli/         command-line interface
src/demo/        deterministic end-to-end demo
test/            node:test automated tests
```
