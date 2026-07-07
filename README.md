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
