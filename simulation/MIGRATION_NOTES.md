# Migration notes: Option 1 → full LTL-D* suffix replanning

Reference: Ren et al. 2024, *LTL-D\** (arXiv:2404.01219), §III, Algs. 1–3, Fig. 6.

## Where we are now

The **prefix** half of LTL-D* is implemented and matches the paper:

- single imaginary goal `s_imag`, every valid accepting state wired to it via a
  virtual edge of cost `cycle_cost[acc]` (`edge_cost()` in `dstar.cpp`);
- one incremental D* Lite g-field to `s_imag` (`compute_shortest_path`);
- the run is reconstructed by descending `g` from the robot to the cheapest
  accepting anchor, then appending that anchor's loop (`dstar_plan`).

The **suffix** half is a stopgap (**Option 1**):

- `make_planner` precomputes each anchor's loop once with a full A* cycle search
  (`astar_cycle_search(acc, true_cycle_only=true)`).
- On replan, `recompute_affected_cycles()` (called from `dstar_replan`) re-runs
  that full A* search for any loop a changed cell touches, refreshes
  `cycle_cost`/`cycle_path`, and couples the new cost back into the prefix.

Option 1 is **correct for blocks** (the cycle no longer walks through obstacles)
but **not incremental** on the suffix — it throws away the speedup that is the
paper's whole point — **and it is not optimal for unblocks**. Its "touched" test
only re-searches a loop when a changed cell lies *on the current cycle path*, so:

- an unblock that opens a **cheaper** route for an existing loop, or
- an unblock that **enables a loop that didn't exist before** (`cycle_cost`
  inf → finite),

is missed, because the changed cell isn't on any current loop. The robot keeps a
valid but **suboptimal** plan until something forces a full rebuild.

This is verified by the test suite (below): `make tests` finds it via the fuzz,
e.g. `seed=1` reaches a state where unblocking `(4,4)` makes anchor `38 (4,5)`
gain a cost-20 loop that the incremental planner still reports as `inf`. The
suite's diagnosis confirms the discrepancy is entirely in `cycle_cost` (suffix),
not the prefix g-field — exactly the gap Option 2 closes.

## Test suite (`tests/replan_tests.cpp`, `make tests`)

The migration has a safety net. After every (re)plan it checks:

- **invariants** (`validate_lasso`): path starts at the robot, no cell blocked,
  all steps (incl. cycle wrap) 4-adjacent, anchor is accepting with a cycle;
- **incremental == rebuild** (`cross_check`): `g(current)` from the incremental
  planner must equal `g(current)` from a fresh `make_planner` on the same
  weights — the strong optimality check;
- **diagnosis** (`diagnose`): on a mismatch, compares `cycle_cost` maps to pin
  the cause to suffix (Option-1 gap) vs prefix (a real bug).

Pinned scenarios all pass; the only failing check is the known Option-1 unblock
gap above. When Option 2 lands, that fuzz failure should disappear — use it as
the acceptance test for the migration.

## What Option 2 (the paper) requires

### 1. Enabling refactor — extract the search engine
`compute_shortest_path` is currently hard-wired to `planner.g/rhs/U/km`. Pull
those into a reusable context so the prefix and every suffix can share one
engine:

```
struct DStarSearch {
    unsigned goal;                       // s_imag (prefix) or s^k_img (suffix k)
    std::unordered_map<unsigned,double> g, rhs;
    std::priority_queue<...> U;
    double km;
    std::unordered_map<unsigned,std::vector<unsigned>> pred_map; // incl. virtual edges
};
```

The current prefix becomes one `DStarSearch{ goal = s_imag }`.
`DStarPlanner` then holds the prefix search + `map<unsigned, DStarSearch> suffix`
keyed by accepting state.

### 2. SUFFIXINITIALIZE (Alg. 1) — replaces the A* precompute in `make_planner`
For each accepting `acc` with a true cycle:
- mint a fresh imaginary id `s^k_img`;
- for every `p` in `PRED(acc)`, add edge `p -> s^k_img` with cost
  `COST(p, acc)` (redirects "step back into acc" to the suffix goal);
- `compute_shortest_path(suffix[acc], sstart=acc, drain_all=true)`;
- `cycle_cost[acc] = suffix[acc].g[acc]`; `cycle_path[acc] =` g-descent
  `acc … s^k_img`.

### 3. SUFFIXREPLAN (Alg. 2) — replaces `recompute_affected_cycles`'s A* call
On `mod` (changed edges):
- for each affected `acc`: apply the weight changes to `suffix[acc]`
  (including the redirected `<u, s^k_img>` edge when `v == acc`),
  `update_vertex`, then `compute_shortest_path(suffix[acc], acc, drain_all=false)`;
- read back the new `cycle_cost[acc]` / `cycle_path[acc]`.

### 4. Coupling (Alg. 3, lines 14–15) — KEEP AS-IS
The block already in `recompute_affected_cycles` that recomputes `rhs(acc)` from
the changed virtual edge `<acc, s_imag>` and requeues it is exactly the paper's
coupling. It carries over unchanged; only the loop-cost *source* (step 3) swaps
from full A* to incremental D*.

## Concrete migration checklist (code pointers)
- [ ] `dstar.cpp` — extract `DStarSearch`; reparametrize `compute_shortest_path`.
- [ ] `dstar.hpp` — add `DStarSearch`; add `suffix` map to `DStarPlanner`.
- [ ] `make_planner` (cycle precompute loop) — replace A* with SUFFIXINITIALIZE.
- [ ] `recompute_affected_cycles` — replace `astar_cycle_search` call with
      SUFFIXREPLAN; keep the COUPLING block.
- [ ] `dstar_replan` — call sites stay the same (still run suffix repair before
      the prefix pass).
- [ ] **Acceptance test**: `make tests && ./replan_tests.out` — the fuzz unblock
      gap (`cycle_cost` inf→finite mismatch) must disappear, and all invariants
      stay green.

## Out of scope (later)
- **β suffix weighting**: total cost = `cost(prefix) + β·cost(suffix)`, β large.
  We implicitly use β = 1 (virtual-edge cost = raw loop cost).
- **Infeasible-task replanning (§IV)**: violation metric `DIST`, auxiliary
  `g_aux`/key with γ≫ weighting for minimally-violating plans. Not started.
- **Suffix search count scaling**: Option 2 maintains one live `DStarSearch`
  (own `pred_map`, `g`/`rhs`, `U`) per accepting state. Today the number of
  accepting states is driven almost entirely by the grid world's size (one
  per cell-ish), which is small and fine. If a future LTL formula makes the
  NBA itself much larger (more accepting states per cell, from richer/more
  formulas), the per-accepting-state suffix count could get expensive. Not
  worth solving now — the planned fix is region abstraction (root
  `README.md`, "Region abstraction (hierarchical planning)"), which collapses
  many product states into fewer, so revisit suffix-count cost only after
  that lands, not before.
