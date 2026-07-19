# TODO

Working backlog. Longer narrative background for anything checked off or
in progress lives in `README.md` (root), `simulation/README.md`, and
`simulation/MIGRATION_NOTES.md` — this file is the flat list.

## Done — partial observability (world/robot map split)

Robot now plans off its own belief map (`robot_map`), not the ground-truth
`world`; only *Dynamic* obstacles are discoverable at runtime (via a
configurable `SensorConfig`), *Static* map structure is fixed and fully
known up front (seeded once, never re-sensed). Implemented across
`grid_world`, new `sensing.hpp/.cpp`, `ts.cpp`, `wpa`, `dstar`/
`dstar_replan`, `grid_vis`, `sim.cpp`, and the `main`/`replan_test`/
`replan_tests` call sites (build-time layouts -> `set_static`, runtime
toggles -> `set_dynamic`). Verified: all five targets build clean
(`-Wall -Wextra`), `replan_tests.out` battery unchanged from baseline
(same pre-existing Option-1 fuzz gap, same Option-2 fix — confirmed by
diffing against the pre-change build via `git stash -u`).

Debug routes added alongside: `output/sensing.log` (`sense`/`reveal`/
`build_state_changes` trace via the existing `dbg()` logger), `events.log`
gains sense/reveal/replan entries, and `grid_vis.cpp` draws a sensor-radius
outline + a "ghost" marker on `Dynamic` cells that are true in `world` but
undiscovered in `robot_map` (toggle with the `D` key) — all functional,
not part of the deferred visual redesign (#7 below).

Follow-up not yet done: `README.md`/`simulation/README.md` still describe
the old `set_blocked`/single-map behavior (e.g. the "build-time obstacles
are not reversible" section) — needs a doc pass to describe `Occupancy`/
`Static`/`Dynamic` instead.

## Backlog (full 7-item list from the planning session)

1. Detection radius — in progress, see above (`sensing.hpp`)
2. Gate replanning on detection trigger — in progress, see above
3. World map vs. robot map — in progress, see above
4. Transition/edge blocks (vs. today's cell blocks) — **not started**,
   no concrete scenario yet justifying it over cell-blocking; revisit
   only when one comes up
5. Object types (`Static`/`Dynamic`, was "walls vs obstacles") — in
   progress, see above
6. NBA-driven map discovery — **deferred**. Only matters once grid
   bounds themselves aren't known upfront (true frontier exploration).
   Today `Static` cells are fixed at construction with no discovery
   mechanism at all; this item is what eventually gives them one
7. Visualizer update (left world / right robot / bottom metrics) —
   **deferred** until the map split lands, so it's built once against
   the finished data model instead of twice

## Longer-term roadmap (carried over from README.md / MIGRATION_NOTES.md)

- Multi-robot skeleton — `ProductBundle` becomes
  `shared_ptr<const ProductBundle>` shared across robots, each with its
  own `DStarPlanner`
- β suffix weighting (`cost(prefix) + β·cost(suffix)`) — not started
- Infeasible-task replanning (LTL-D* §IV) — `DIST` violation metric +
  `g_aux` — not started
- Region abstraction (hierarchical planning) — collapse `(region,
  nba_state)` into one product state; revisit suffix-count cost
  (MIGRATION_NOTES.md) only after this lands
- Raytrace-based sensing model — slots into `sense()`'s existing
  signature (`SensorMetric`/`SensingCadence` already scaffolded for it)
- Design the task-allocation layer proper (Li et al. 2023 / Luo &
  Zavlanos 2021) — sits above the per-robot planners, most valuable
  once the multi-robot skeleton exists
- Cycle cache for constantly-changing worlds (open question, not
  designed yet)
