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
gains sense/reveal/replan entries, and a sensor-radius outline on the robot
pane (toggle with the `D` key).

Fixed after initial landing: sensing was gated on `stepped` (only ran when
the robot actually advanced a cell), which deadlocked — a robot halted by a
blocked path never steps, so it could never re-sense an unblock and would
stay stuck forever even after the cell reopened. Sensing now runs on its
own timer (`sense_elapsed`/`sense_interval` in `grid_vis.cpp`), fully
decoupled from movement; `SensingCadence::EveryTick` means every
simulation tick regardless of motion, `OnMove` is the old (now correctly
named) step-gated behavior.

Follow-up not yet done: `README.md`/`simulation/README.md` still describe
the old `set_blocked`/single-map behavior (e.g. the "build-time obstacles
are not reversible" section) — needs a doc pass to describe `Occupancy`/
`Static`/`Dynamic` instead.

## Done — dual-pane visualizer + metrics (#7)

`grid_vis.cpp`'s `dynamic_visulizer` now renders two card-style panes side
by side — **World (ground truth)**, click-to-toggle `Dynamic` obstacles,
left — and **Robot belief**, the actual planning input with the live
path/lasso overlay, right — plus a bottom metrics strip. `Static`/`Dynamic`
occupancy render in distinct colors (charcoal wall vs. rust obstacle) on
both panes. Prefix and loop are drawn as separate colors on the path line
(blue/purple normally, shifting per replan, collapsing to red while
blocked) with the actual cell sequence for each printed as text under the
numeric metrics (prefix length, cycle length, total, replan count, last
replan wall-clock time, live `g(current)`). Visual style (cards, rounded
inset cells, circular label badges, line-based path instead of stacked
translucent rectangles) is centralized in the `theme` namespace at the top
of `grid_vis.cpp` for easy retuning. Robot marker + sensor-radius outline
are currently robot-pane-only (commented out, not deleted, on the world
pane — easy to re-enable).

## Done — sim testing/UX enhancements (prerequisite for the discovery work below)

All landed in `grid_vis.cpp`'s `dynamic_visulizer`, keyboard-toggle based
(no `raygui` dependency added), with an on-screen status line + legend so
toggle states are visible while recording. Verified: `make sim`/`main`/
`replantest`/`tests` all build clean, `replan_tests.out` unchanged from
baseline, `sim.out` runs and `output/metrics.csv` populates with real rows.

- [x] **Start/stop (pause) control** (`SPACE`) — freezes movement +
      sensing; rendering/clicking/mode-toggles keep working while paused.
- [x] **In-sim `SuffixMode` toggle** (`M`) — rebuilds the planner from the
      WPA's current edge weights via `make_planner`, then re-runs
      `dstar_plan` from the robot's current state.
- [x] **In-sim `ReplanMode` toggle** (`N`) — flips `planner.mode` for
      subsequent replans.
- [x] **Obstacle-type placement toggle** (`T`) — click places `Static`
      instead of `Dynamic`. Note (as expected): a `Static` cell placed
      *after* the product is built won't remove its graph node — matches
      the existing documented build-time-blocking limitation, fine since
      map-building is meant to happen paused/early.
- [x] **On-screen status line + legend** — control readout (`draw_status_line`)
      plus Static/Dynamic/per-label color swatches (`draw_legend`), both
      above the two panes.
- [x] **Single-step while paused** (`RIGHT` arrow) — pre-loads `elapsed`/
      `sense_elapsed` past their thresholds so the *same* movement+sense
      logic fires exactly once, no separate code path needed.
- [x] **Adjustable step speed** (`+`/`-`, i.e. `KEY_EQUAL`/`KEY_MINUS`) —
      `step_interval` now mutable, clamped `[0.05s, 2.0s]`.
- [x] **Save/load the world layout** (`F5`/`F9`) — plain-text format
      (`WIDTH`/`HEIGHT`/`LABEL`/`STATIC`/`DYNAMIC` lines) to
      `output/world_layout.txt`; `GridWorld` gained `set_free()` (force-clear
      regardless of current type) to support resetting before a load.
      World-only (ground truth) — the robot still only learns via `reveal()`.
- [x] **Screenshot capture** (`F2`) — `TakeScreenshot()` to
      `output/screenshot_NNN.png`.
- [x] **Live sensor-radius adjustment** (`[`/`]`) — `sensor_cfg.radius`,
      clamped `[0.5, 10.0]`; `dynamic_visulizer`/`SensorConfig` parameter is
      now a non-const reference to support this.
- [x] **Metrics-to-CSV dump** — `output/metrics.csv`, one row per
      simulated step (`step_index,pos_x,pos_y,prefix_len,cycle_len,
      total_len,replan_count,last_replan_ms,g_current`).
- [x] **On-screen color legend** — folded into the status-line area
      (`draw_legend`), not a separate lower-priority pass.

Explicitly skipped (named, not forgotten): manual robot teleport and
undo-last-edit. Neither blocks the discovery work, and both add
state-management complexity for a convenience that may not get reached for.

## In progress — NBA-driven incremental map discovery

Next after the sim enhancements above (chosen over the multi-robot
skeleton: less invasive, and a literature check below narrowed it to a
specific, defensible angle rather than the more crowded "automaton-driven
map discovery" umbrella).

Only matters once grid bounds themselves aren't known upfront (true
frontier exploration) — today `Static` cells are fixed at construction
with no discovery mechanism at all; this is what gives them one, and is
also the natural continuation of the "fixed graph, changing costs" work
already done for `Dynamic` cells (root `README.md`, "Fixed graph, changing
costs").

**Novelty check (2026-07-18)**, done before committing to this: neither
LTL-D* (Ren et al. 2024) nor the task-allocation paper (Li et al. 2025)
address incremental automaton-state minting for unknown/growing maps —
LTL-D*'s partial-observability benchmark uses a fully pre-built product
over a known-size grid, and Li et al.'s Conclusion names unknown
environments as explicit unaddressed future work. But a broader web search
found this *is* an active area in the wider literature (see root
`README.md`'s "Related external work" list under "Frontier-based
exploration") — none of it pairs the idea with D* Lite-style incremental
replanning specifically, though, which is the angle this codebase already
has working. That pairing is the scoped contribution to aim for, not
"NBA-driven discovery" as a broad claim.

Full design plan (with file/function-level detail) lives in root
`README.md` under "Frontier-based exploration" → "Design plan
(2026-07-18, not yet implemented)". Flat checklist below; no
implementation started yet.

Decision locked in: labels (`a`/`b`/`c`/`d`) are known upfront by default
(`SensorConfig::labels_known_upfront = true`), built so it can be toggled
to "labels also hidden until sensed" later without a redesign.

**Phase 1 (v1 — discovery + incremental minting + D\* integration;
independently testable — robot idles/reports infeasible until enough map
is discovered):**
- [ ] `grid_world.hpp/.cpp`: `Occupancy` gains `Unknown`;
      `SensorConfig` gains `labels_known_upfront`
- [ ] `sensing.hpp/.cpp`: `seed_frontier_belief()` (frontier-mode
      seeding, `seed_from_static()` stays untouched for bounded mode);
      `reveal()` resolves `Unknown -> {Static,Free,Dynamic}` and returns
      `RevealEvent{Pos, newly_discovered}` instead of plain `vector<Pos>`
- [ ] `ts.cpp`: per-position minted-states index; `discover_cell()` —
      mint nothing for `Static`, mint `(p,q)` + wire edges via
      `prod->new_state()`/`new_edge()` for `Free`/`Dynamic`; extend
      `pred_map` incrementally
- [ ] `dstar.cpp`/`dstar_replan.cpp`: seed `rhs` for brand-new states
      from their new edges (reuses `compute_shortest_path`/
      `update_vertex` unchanged otherwise)

**Phase 2 (v2 — active frontier-seeking):**
- [ ] Frontier-selection policy: BFS from `current` over
      discovered-and-traversable cells to the nearest one bordering
      `Unknown`, steer there when `g(current) == inf`

## Backlog

- Transition/edge blocks (vs. today's cell blocks) — **not started**,
  no concrete scenario yet justifying it over cell-blocking; revisit
  only when one comes up

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
