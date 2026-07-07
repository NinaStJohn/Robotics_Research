# Migration notes: Option 1 → full LTL-D* suffix replanning

Reference: Ren et al. 2024, *LTL-D\** (arXiv:2404.01219), §III, Algs. 1–3, Fig. 6.

## Status: migration complete (Option 2 implemented, both options kept as a runtime toggle)

Both the **prefix** and **suffix** halves of LTL-D* are implemented and match the
paper. The suffix half has two interchangeable strategies, selected per-planner
via `SuffixMode` (`dstar.hpp`) so they can be A/B'd on the same scenario instead
of only compared via git history:

- **`SuffixMode::OPTION1_ASTAR`** (default) — stopgap: `make_planner` precomputes
  each anchor's loop once with a full A* cycle search
  (`astar_cycle_search(acc, true_cycle_only=true)`); on replan,
  `recompute_affected_cycles()` re-runs that full A* search for any loop a
  changed cell *touches*, refreshes `cycle_cost`/`cycle_path`, and couples the
  new cost back into the prefix. Correct for blocks, but not incremental, and
  **not optimal for unblocks**: its "touched" test only re-searches a loop when
  a changed cell lies on the *current* cycle path, so an unblock that opens a
  cheaper route, or enables a loop that didn't exist before (`cycle_cost`
  inf → finite), is missed until something forces a full rebuild. This is the
  gap the fuzz test in `replan_tests.cpp` pins (`seed=1`, unblocking `(4,4)`
  should let anchor `38 (4,5)` gain a cost-20 loop; Option 1 still reports `inf`).

- **`SuffixMode::OPTION2_INCREMENTAL`** (paper-faithful) — one D* Lite search
  per accepting state (`DStarSearch`, goal = a fresh imaginary node `s^k_img`),
  built once via **SUFFIXINITIALIZE** (Alg. 1, in `make_planner`) and repaired
  incrementally via **SUFFIXREPLAN** (Alg. 2, `suffixreplan()` in `dstar.cpp`)
  on every replan. Unlike Option 1's touched-test, `suffixreplan` applies the
  changed edge costs to *every* accepting state's suffix search unconditionally
  — the real graph is shared across all of them — then lets
  `compute_shortest_path` do incremental (not full) repair. This closes the
  Option-1 gap: `replan_tests.cpp`'s battery re-runs entirely under
  `OPTION2_INCREMENTAL` and the same fuzz seed that fails under Option 1
  **passes** under Option 2.

## How it works

### Shared search engine (`DStarSearch`, `dstar.hpp`)
`compute_shortest_path`/`calculate_key`/`update_vertex` all operate on a
`DStarSearch` (goal, `g`, `rhs`, `km`, `U`, `pred_map`) rather than being
hard-wired to the planner. `DStarPlanner` holds one `prefix` search (goal =
`s_imag`) plus a `map<unsigned, DStarSearch> suffix` keyed by accepting state,
populated only under `OPTION2_INCREMENTAL`.

Each `DStarSearch` carries a `kind` (`PREFIX`/`SUFFIX`) and, for suffix
searches, a `redirect_target` (the anchor `acc` it closes a loop for).
`edge_cost()` (`dstar.cpp`) resolves the virtual goal-closing edge generically
from these: `PREFIX` uses `cycle_cost[from]`; `SUFFIX` uses the real edge cost
from `from` into `redirect_target`. Both are **added** edges, never replacing
a real one — safe because `g(goal) == 0` always dominates going around the
loop again, so the redirect edge is never worse than continuing through the
anchor's own (still-converging) g value.

### SUFFIXINITIALIZE (Alg. 1) — `make_planner`, `OPTION2_INCREMENTAL` branch
For each accepting `acc`:
- mint a fresh imaginary id `s^k_img`;
- `suffix[acc].pred_map` = the real reverse graph, plus `PRED(acc) -> s^k_img`
  (every real predecessor of `acc` also gets an edge straight to the goal —
  "step back into acc" redirected to closing this search's loop);
- `compute_shortest_path(suffix[acc], sstart=acc, sgoal=s^k_img, drain_all=true)`;
- `cycle_cost[acc] = suffix[acc].g[acc]`; `cycle_path[acc]` = g-descent from
  `acc` to `s^k_img` (`suffix_cycle_path()`, mirrors `dstar_plan`'s prefix
  descent, terminating when the virtual closing edge is the best option).

Verified against Option 1 in `replan_tests.cpp`'s `check_suffix_init_parity`:
identical `cycle_cost` on both an open grid and an obstacle-heavy layout.

### SUFFIXREPLAN (Alg. 2) — `suffixreplan()` in `dstar.cpp`
On a changed edge cost (`cold` → `new_cost`, same values `dstar_replan` already
applied to the prefix):
- for every accepting state's suffix search: apply the same
  `repair_rhs_for_changed_edge` ({43-46}) used for the prefix, `update_vertex`,
  then `compute_shortest_path(suffix[acc], acc, s^k_img, drain_all=false)`;
- read back `cycle_cost[acc]`/`cycle_path[acc]` from the repaired search.

`repair_rhs_for_changed_edge` is shared (not duplicated) between the prefix's
replan loop (`dstar_replan.cpp`) and `suffixreplan`'s per-accepting-state loop
— same {43-46} logic, parameterized on which `DStarSearch` it repairs.

### Coupling (Alg. 3, lines 14–15) — unchanged
The block that recomputes `rhs(acc)` from the changed virtual edge
`<acc, s_imag>` and requeues it is identical in `recompute_affected_cycles`
(Option 1) and `suffixreplan` (Option 2) — copied over verbatim, since it was
already paper-correct.

## Test suite (`tests/replan_tests.cpp`, `make tests`)

- **Tier 0** `check_suffix_init_parity` — Option 2's initial build vs Option 1's,
  same `cycle_cost` on multiple world layouts.
- **Tier 1** `validate_lasso` — structural invariants (adjacency, no blocked
  cells, valid accepting anchor).
- **Tier 2** `cross_check` — incremental `g(current)` must equal a fresh
  `make_planner` rebuild on the same weights, parameterized by `SuffixMode` so
  each mode is checked against its own from-scratch rebuild.
- **Tier 3/4** pinned scenarios + seeded fuzz, run once under
  `OPTION1_ASTAR` (default) and again under `OPTION2_INCREMENTAL`.

Current result: the full battery passes under both modes **except** the known
Option-1 fuzz gap (expected — that's the bug Option 2 exists to fix, and the
same seed passes in the Option-2 re-run).

## CLI toggle (`apps/replan_test.cpp`)
`./replantest.out --suffix=option1|option2 <robot_step> <bx> <by> ...` — lets
you script the same obstacle-toggle scenario under either suffix strategy
without touching source. `--suffix=` can appear anywhere in the argument list.

## Out of scope (later)
- **β suffix weighting**: total cost = `cost(prefix) + β·cost(suffix)`, β large.
  We implicitly use β = 1 (virtual-edge cost = raw loop cost).
- **Infeasible-task replanning (§IV)**: violation metric `DIST`, auxiliary
  `g_aux`/key with γ≫ weighting for minimally-violating plans. Not started.
- **Suffix search count scaling**: Option 2 maintains one live `DStarSearch`
  (own `pred_map`, `g`/`rhs`, `U`) per accepting state, and `suffixreplan`
  touches all of them on every replan (not just ones near the change — that's
  what makes it complete, per the Option-1 gap it fixes). Today the number of
  accepting states is driven almost entirely by the grid world's size (one
  per cell-ish), which is small and fine. If a future LTL formula makes the
  NBA itself much larger (more accepting states per cell, from richer/more
  formulas), the per-accepting-state suffix count — and the per-replan cost of
  touching all of them — could get expensive. Not worth solving now — the
  planned fix is region abstraction (root `README.md`, "Region abstraction
  (hierarchical planning)"), which collapses many product states into fewer,
  so revisit suffix-count cost only after that lands, not before.
