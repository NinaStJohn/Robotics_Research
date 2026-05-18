
# Robotics LTL-D* from simulation to implemtation #

This repository is deteicated to exploring LTL's applicaiton in robotics. 


### References ###

the spot github that was used, in Ubuntu
```
// required build dependencies
sudo apt update
sudo apt install -y \
  build-essential git pkg-config \
  autoconf automake libtool \
  flex bison \
  libbdd-dev libgmp-dev libboost-all-dev \
  python3-dev swig

// clone with submodules
cd ~
git clone --recurse-submodules https://gitlab.lre.epita.fr/spot/spot.git
cd spot

// bootstrap
autoreconf -vfi

// configure
./configure --prefix=/usr/local

// build
make -j$(nproc)

// install
sudo make install
sudo ldconfig

```
this is a warning for furture me reference
```
===================================================================
 This is a development version of Spot: Assertions and debuging
 code are enabled by default.  If you find this too slow or
 plan to do some benchmarking, run configure with --disable-devel.
===================================================================
```

For getting the visulization for LTL
```
sudo apt install graphviz xdot

// dot viewer example
ltl2tgba -f "GFa & GFb" --dot | xdot -

// open pdf example
xdg-open automaton.pdf
```

The makefile right now has a sim and a test flag. Use the test to see if spot is working. Sim is our grid_world simualtion.

Visulization for LTL
```
sudo apt install graphviz xdot
./test 2> aut.dot
xdot aut.dot
```
Visulization for Gridworld (raylib)
https://github.com/raysan5/raylib/wiki/Working-on-GNU-Linux

---

## Future Directions (Hypothetical)

### Replanning modes
The D* planner supports two modes: incremental (`DSTAR_INCREMENTAL`) and full recompute (`FULL_RECOMPUTE`). For a fixed-size grid, all scenarios — obstacles appearing, cells being unblocked, internal discovery — are cost changes on an existing graph and are handled incrementally. Full recompute is kept as a correctness baseline.

### Frontier-based exploration
If the grid itself grows (expanding map, frontier-based exploration), new grid cells mean new product states that were never allocated. That is a topology change, not a cost change. The product automaton must be rebuilt and D* tables reset. This is the one scenario that requires `FULL_RECOMPUTE`. For a fixed-size map where traversability is unknown but the grid bounds are known, pre-allocate all product states at construction and treat unknown cells as high-cost — D* handles discovery incrementally.

### Region abstraction (hierarchical planning)
For computational efficiency, grid cells that share the same LTL label can be grouped into regions (e.g. cells (0,0)–(4,0) all labeled "A" become one abstract region). Two approaches:

- **Label grouping**: cells share a label so NBA transitions are identical, but product states are still per-cell. Cheaper NBA computation, D* unchanged.
- **True region abstraction**: collapse (region, nba_state) into a single product state. Smaller product automaton, but requires well-defined inter-region edge costs and a separate local planner for within-region navigation. This is hierarchical planning — abstract D* picks the next region, local planner picks the cell within it.

True region abstraction is a natural extension once the single-robot case is solid. It also pairs well with multi-robot scaling (fewer states = much cheaper joint product automaton).

### Multi-robot extension
The long-term goal is LTL-based task allocation for heterogeneous multi-robot systems (see `papers/ltl_fasktrackallocation.pdf`, `papers/ltl_fasttrackltl.pdf`). The WPA is currently single-robot. The `ProductBundle` is designed to eventually be shared across robots via `shared_ptr<const ProductBundle>`, with per-robot `DStarPlanner` instances.

## Random Thoughs
For constantly changing worlds a cycle cache?
Add vision 'cone' in simulation - or like lidar detection or something