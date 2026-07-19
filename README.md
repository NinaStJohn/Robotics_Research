# Robotics LTL-D\* — from simulation to implementation

Exploring LTL's application in robotics. A single-robot LTL planner with **D\*
Lite incremental replanning** over a product automaton (transition system ×
NBA), built as a stepping stone toward LTL-based task allocation for
heterogeneous multi-robot systems.

- **Module layout, build targets, how to run** → [`simulation/README.md`](simulation/README.md)
- **Replanning status + the Option-2 (paper-faithful) migration plan** → [`simulation/MIGRATION_NOTES.md`](simulation/MIGRATION_NOTES.md)
- **Reference papers** → `papers/` (LTL-D\* core: Ren et al. 2024, arXiv:2404.01219)

## Status

- **Prefix replanning**: implemented and **validated** (test suite: `make tests`).
- **Suffix (cycle) replanning**: both implemented and validated — Option-1
  stopgap (full A\* re-search of touched loops) and Option-2 paper-faithful
  incremental `SUFFIXREPLAN`, selectable via `SuffixMode` (default: Option 1)
  — see the migration notes for how they compare.
- **Known limitation** (validated): cells blocked *before* the product is built
  have no graph node and cannot be unblocked/traversed at runtime; toggle
  obstacles by clicking for reversible behavior. Details in `simulation/README.md`.

## Building Spot (dependency)

Spot is cloned into `spot/` (a standalone upstream clone tracking `origin/next`,
not a submodule) and installed system-wide; the Makefile links it via
`pkg-config`.

```sh
# required build dependencies
sudo apt update
sudo apt install -y \
  build-essential git pkg-config \
  autoconf automake libtool \
  flex bison \
  libbdd-dev libgmp-dev libboost-all-dev \
  python3-dev swig

# clone with submodules
cd ~
git clone --recurse-submodules https://gitlab.lre.epita.fr/spot/spot.git
cd spot

autoreconf -vfi                       # bootstrap
./configure --prefix=/usr/local       # configure
make -j$(nproc)                        # build
sudo make install && sudo ldconfig     # install
```

Note: Spot's default build enables assertions/debug code (slower). For
benchmarking, `./configure --disable-devel`.

## Visualization tools

```sh
sudo apt install graphviz xdot

# LTL automaton -> dot viewer
ltl2tgba -f "GFa & GFb" --dot | xdot -

# the `test` target prints an automaton to stderr as dot
./test.out 2> aut.dot && xdot aut.dot
```

Gridworld visualization uses raylib:
<https://github.com/raysan5/raylib/wiki/Working-on-GNU-Linux>

---

## Next steps (prioritized, as of the Option-2 migration landing)

Four candidate directions were on the table; recommended order and why:

1. **Fixed graph, changing costs** — pre-allocate a product state per cell so
   block/unblock is always a cost change, never a topology change. Fixes the
   known build-time-obstacle limitation (see Status above). Recommended
   *before* the multi-robot skeleton (#2): retrofitting this after robots
   already share a graph is more disruptive than doing it first.
   - Natural follow-on once this lands: **NBA-driven incremental node
     discovery** (see "Frontier-based exploration" below) — mint product
     states for newly-seen cells on demand, using the NBA's transition
     function to determine their edges, instead of a full `FULL_RECOMPUTE`
     rebuild per discovery.
2. **Multi-robot skeleton** — `ProductBundle` becomes
   `shared_ptr<const ProductBundle>` shared across robots, each with its own
   `DStarPlanner`. The direct pivot toward the task-allocation papers, but
   inherits the build-time-blocking asymmetry if done before #1.
3. **Finish single-robot LTL-D\* fidelity** — β suffix weighting
   (`cost(prefix) + β·cost(suffix)`) and infeasible-task replanning (§IV,
   the `DIST` violation metric + `g_aux`). Deepens paper coverage; fully
   decoupled from the multi-robot work, so it can slot in anytime or be
   skipped.
4. **Design the task-allocation layer** — map Li et al. 2023
   (`papers/ltl_fasktrackallocation.pdf`) or Luo & Zavlanos 2021
   (`papers/ltl_fasttrackltl.pdf`) onto a concrete allocation design sitting
   above the per-robot planners. No code yet — most valuable once #2 exists.

## Future Directions (hypothetical)

### Replanning modes
The D\* planner supports incremental (`DSTAR_INCREMENTAL`) and full recompute
(`FULL_RECOMPUTE`). For a fixed-size grid, obstacles appearing, cells being
unblocked, and internal discovery are all cost changes on an existing graph and
are handled incrementally. Full recompute is kept as a correctness baseline (and
is what the test suite cross-checks against).

### Fixed graph, changing costs (fixes the build-time-blocking limitation)
For a fixed-size map where traversability is unknown but the grid bounds are
known, pre-allocate a product state for **every** cell at construction and treat
blocked/unknown cells as high-cost edges. Then block/unblock is purely a cost
change D\* handles incrementally — and the current asymmetry (build-time blocks
remove nodes, runtime blocks re-weight) disappears.

### Frontier-based exploration
If the grid itself grows (expanding map), new cells mean new product states never
allocated — a *topology* change, not a cost change. That requires rebuilding the
product and resetting D\* tables (`FULL_RECOMPUTE`).

**Incremental node discovery via the NBA**, as a cheaper alternative to a full
rebuild per newly-seen cell: when the robot discovers a previously-unknown
cell, mint its product state(s) on demand instead of re-running
`build_product_from_world_robot_ltl` — for each NBA state already reachable at
that grid position, query the NBA's transition function (`is_accepting`,
successor edges) against the new cell's label to determine which outgoing
edges exist, then wire the new state to whichever *already-discovered*
neighbor states it connects to (edges to still-undiscovered neighbors stay
unresolved until those neighbors are minted too). This keeps the graph growing
node-by-node instead of rebuilding wholesale, so D\* only needs
`update_vertex`/`compute_shortest_path` on the newly-added states — much
closer to true incremental replanning than `FULL_RECOMPUTE`. Depends on the
fixed-graph work above for the "how do I represent an as-yet-unknown cell"
question, and pairs naturally with region abstraction below once the frontier
gets large.

**Novelty check (2026-07-18):** incremental product-automaton construction
paired with frontier-based exploration under LTL is an active research area,
not open ground — see "Related external work" below. None of it pairs this
with D\* Lite-style incremental replanning specifically (they lean on
sampling-based, potential-field, or regret-based planners instead), which is
the angle this codebase already has working via LTL-D\*. That pairing —
NBA-driven incremental state minting feeding an already-incremental D\* Lite
search — is the narrower, more defensible contribution to aim for here,
rather than "automaton-driven map discovery" in general.

#### Related external work (not in `papers/`, found via literature check)
- [Temporal-Logic-Aware Frontier-Based Exploration](https://arxiv.org/pdf/2602.18951)
- [Motion Planning Under Temporal Logic Specifications In Semantically Unknown Environments](https://arxiv.org/pdf/2511.03652)
- [To Explore or Not to Explore: Regret-Based LTL Planning in Partially-Known Environments](https://arxiv.org/pdf/2204.00268)
- [Guaranteeing High-Level Behaviors while Exploring Partially Known Maps](https://www.roboticsproceedings.org/rss08/p48.pdf)
- [Hybrid and Oriented Harmonic Potentials for Safe Task Execution in Unknown Environment](https://arxiv.org/pdf/2306.07537)

#### Design plan (2026-07-18, not yet implemented)

Labels (`a`/`b`/`c`/`d`) are known upfront (like a floorplan with marked
target rooms but unknown terrain), with the flag built so this can be
toggled to "labels also hidden until sensed" later — see `SensorConfig`
below.

Good news that shrinks the scope: `spot::product()` (the algorithm) is
one-shot, but the underlying `spot::twa_graph` it returns supports
incremental `new_state()`/`new_edge()` at any time — the same calls
`world_to_ts` already uses to build the TS by hand. So this isn't "replace
spot with a hand-rolled graph," it's "stop calling `spot::product()`, and
grow a `spot::twa_graph` ourselves, one discovered cell at a time." `WPA`'s
interface (`neighbors_ext`, `pos_of`, `is_accepting`, `state_exit_weight`,
etc.) barely changes — it already just reads `prod->out(state)` and
queries maps keyed by product-state-id, and doesn't care whether the graph
was populated in one shot or grown lazily.

**Phase 1 (v1 — discovery + incremental minting + D\* integration):**

1. `grid_world.hpp`/`.cpp`: `Occupancy` gains a 4th value, `Unknown` — the
   default for every cell in a frontier-mode `robot_map`, since even wall
   layout isn't known until sensed now. `SensorConfig` gains
   `bool labels_known_upfront = true;`.
2. `sensing.hpp`/`.cpp`: new `seed_frontier_belief(world, labels_known_upfront)
   -> GridWorld` (same bounds as `world`, everything `Unknown`, labels
   copied in only if the flag is set) replaces `seed_from_static` for
   frontier mode — bounded mode keeps `seed_from_static` unchanged, no
   regression to the existing feature. `reveal()` gets heavier: for a still-
   `Unknown` cell it must resolve `Unknown -> {Static, Free, Dynamic}` from
   `world` truth (and copy the label too, if `!labels_known_upfront`).
   Return type needs to distinguish *first-time-discovered* cells (which
   need minting, step 3) from *already-known-Dynamic-cell-flipped* cells
   (which just need the existing cost-reweight path) — proposing
   `RevealEvent{Pos, bool newly_discovered}` instead of the current plain
   `vector<Pos>`.
3. Incremental minting (likely lands in `ts.cpp`, which already owns the
   NBA-transition/label-BDD logic this needs): maintain a per-position
   index (`unordered_map<Pos, vector<unsigned>>`) of which NBA states are
   currently minted at each cell, so a newly discovered cell's
   already-minted neighbors can be found without scanning the whole graph.
   `discover_cell(p, ...)`: if `p` resolves `Static`, mint nothing, ever
   (matches today's rule, just decided later than construction). If
   `Free`/`Dynamic`: for every already-minted neighboring state `(p', q')`,
   compute the NBA successor `q = δ(q', label(p))`, mint `(p, q)` via
   `prod->new_state()` if new (reusing `ProductBundle`'s existing
   `ts_state_to_pos`/`prod_state_to_nba`/`pos_nba_to_prod` bookkeeping —
   barely changes), wire the edge via `prod->new_edge()` the same way
   `world_to_ts` already builds edges. Extend `pred_map` incrementally too
   (`pred_map[q].push_back(p'q')`), since D* Lite depends on it.
4. `dstar.cpp`/`dstar_replan.cpp`: barely changes. Reuses
   `compute_shortest_path`/`calculate_key`/`update_vertex` as-is — a newly
   minted state already defaults to `g=rhs=∞` (`unordered_map` lookup), and
   folds in via `update_vertex` exactly like a cost-changed state does
   today. The only new logic is seeding `rhs` for brand-new states from
   their new edges, structurally the same shape as `dstar_replan`'s
   existing loop.

**Phase 2 (v2 — active frontier-seeking):**

5. Frontier-selection policy (genuinely new behavior, no existing analog):
   when `g(current) == ∞` (nothing discovered yet leads to an accepting
   anchor), the robot needs to move toward the nearest *undiscovered* cell
   to reveal more map, instead of sitting still. A separate BFS over
   discovered-and-traversable cells, targeting the nearest one bordering
   `Unknown` territory — it doesn't know or care about the LTL task at all,
   purely an information-gain search.

v1 (1-4) is independently testable — the robot just idles/reports
infeasible until enough of the map is discovered by luck of the sensor
radius passing over new cells. v2 (5) is what makes it actually explore.

### Region abstraction (hierarchical planning)
Group cells sharing an LTL label into regions. Either keep per-cell product
states with shared labels (cheaper NBA, D\* unchanged), or collapse
`(region, nba_state)` into one product state (smaller automaton, needs
inter-region costs + a local within-region planner). Pairs well with multi-robot
scaling.

### Multi-robot extension
Long-term goal: LTL task allocation for heterogeneous multi-robot systems (see
`papers/ltl_fasktrackallocation.pdf`, `papers/ltl_fasttrackltl.pdf`). The WPA is
single-robot today; `ProductBundle` is meant to become
`shared_ptr<const ProductBundle>` shared across robots, with a per-robot
`DStarPlanner`.

## Random thoughts
- Cycle cache for constantly-changing worlds?
- Add a vision cone / lidar-style detection in the simulation.

## Commands: testing / comparing outputs

All from the `simulation/` directory (`cd simulation`).

```bash
# GUI — left-click a cell to toggle an obstacle, watch the robot replan live
make sim && ./sim.out

# headless scripted scenario — Option 1 (default) vs Option 2, same scenario
make replantest
./replantest.out 6 2 1 2 0                     # Option 1 (stopgap A*)
./replantest.out --suffix=option2 6 2 1 2 0    # Option 2 (incremental SUFFIXREPLAN)
# --suffix= can appear anywhere in the args; args are:
#   [--suffix=option1|option2] <robot_step> <bx> <by> [<bx2> <by2> ...]

# correctness/regression suite — runs the full battery under BOTH SuffixModes
# in one invocation; exit code = number of failing checks
make tests && ./replan_tests.out [fuzz_seed] [fuzz_steps]
# e.g. re-run the pinned fuzz case that distinguishes Option 1 from Option 2:
./replan_tests.out 1 60
```

`replan_tests.out` output is split into labelled `TEST [...]` blocks — the
`[opt2] ...` labelled ones are the Option-2 re-run of the same scenarios run
unlabelled (Option 1, the default) earlier in the same output, so diffing the
two halves shows exactly where the two strategies agree/disagree.
