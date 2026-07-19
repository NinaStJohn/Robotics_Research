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
