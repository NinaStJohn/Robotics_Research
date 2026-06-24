# Simulation module

Single-robot LTL planner with **D\* Lite incremental replanning** over a product
automaton (transition system × NBA). A stepping stone toward LTL-based task
allocation for heterogeneous multi-robot systems. Core reference: Ren et al.
2024, *LTL-D\** (arXiv:2404.01219) — see [MIGRATION_NOTES.md](MIGRATION_NOTES.md)
for how far along that implementation is.

## Pipeline at a glance

```
GridWorld  ──►  TS (twa_graph)  ─┐
                                 ├─►  ProductBundle  ──►  WPA  ──►  D* planner  ──►  lasso
LTL formula ─►  NBA (twa_graph) ─┘     (TS × NBA)      (weights)   (make_planner)   (prefix·cycleᵂ)
```

The robot's run is a **lasso**: a one-time **prefix** (g-descent from the robot
to the cheapest accepting anchor) followed by a **cycle** (the repeating
accepting loop). When the world changes, only the affected part is replanned.

## Source layout (`src/`, `include/`)

| File | Role |
|---|---|
| `grid_world` | grid model: bounds, obstacles (`set_blocked`), labels, neighbors |
| `robot` | robot state / start position |
| `ts` | builds the TS from the world+robot and the product automaton (`build_product_from_world_robot_ltl`) |
| `wpa` | **Weighted Product Automaton**: wraps the product, attaches edge weights, exposes `neighbors_ext`, `pos_of`, `is_accepting`, `set_state_exit_weight` |
| `dstar` | D* Lite core: `make_planner` (initial full build), `compute_shortest_path`, `dstar_plan` (lasso reconstruction), `astar_cycle_search` (suffix loops), `recompute_affected_cycles` (Option-1 suffix repair) |
| `dstar_replan` | `detect_changed_states` (cells → product states) and `dstar_replan` (incremental edge update + prefix repair + cycle repair) |
| `grid_vis` | raylib animator: draws the grid, animates the robot along the lasso, handles click-to-toggle obstacles, per-replan colors |
| `LTL_vis` | DOT export of NBA / TS / product for inspection |
| `debug_log` | `dbg("name.log")` → append-mode file logger under `output/` |

## Build targets (`make <target>`)

| Target | Output | What it is |
|---|---|---|
| `sim` | `sim.out` | interactive GUI (raylib): watch the robot, click to add/remove obstacles |
| `replantest` | `replantest.out` | **headless** harness: script obstacle toggles from the CLI (no clicking) |
| `tests` | `replan_tests.out` | correctness suite (invariants + incremental-vs-rebuild + fuzz) |
| `main` | `main.out` | minimal entry point |
| `test` | `test.out` | standalone spot/LTL demo (`test.cpp`) |

> `tests` is `.PHONY` (it collides with the `tests/` directory).

## Running

```bash
make sim && ./sim.out                 # GUI; left-click a cell to toggle an obstacle
make replantest && ./replantest.out 0 5 2     # headless: robot at step 0, toggle (5,2)
make tests && ./replan_tests.out      # exit code = number of failing checks
```

Headless harness args: `./replantest.out <robot_step> <bx> <by> [<bx2> <by2> …]`
— place the robot at `<robot_step>` on the path, then toggle each cell with a
full replan between. Test suite args: `./replan_tests.out [fuzz_seed] [fuzz_steps]`.

## Debug output (`output/`, gitignored)

Written by `dbg()` during a run: `reachability.log` (topological reach to the
accepting set), `gfield_initial.log` (g/rhs after the initial full build),
`replan.log` (per-replan g/rhs + edge updates), `events.log` (robot steps +
clicks). The planner also prints the prefix g-descent and the cycle to stdout.

## Status / what's next

- **Prefix replanning**: implemented, matches LTL-D* (verified by the test suite).
- **Suffix (cycle) replanning**: **Option 1** stopgap (full A\* re-search of
  touched loops). Correct but not incremental, and it misses unblocks that
  *enable a new* loop — see [MIGRATION_NOTES.md](MIGRATION_NOTES.md) for the
  Option-2 (paper-faithful `SUFFIXREPLAN`) migration plan.
- **Not started**: β suffix weighting; infeasible-task replanning (LTL-D\* §IV).

## Spot / LTL cheatsheet

`G` = always, `F` = eventually; `twa_product(a, b)` takes the product.
Key spot types: `spot::formula`, `spot::translator`, `spot::twa_graph_ptr`,
`spot::print_dot`. Acceptance is transition-based internally (outgoing edges of
accepting states carry `acc={0}`); use `is_accepting(dst)` for neighbors.
