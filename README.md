
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
Add vision 'cone' in simulation - or like lidar detection or something <- do this

## TODO by next meeting

### Phase 1 — Get D* producing the path (replace A*)

**1a. Implement `reconstruct_lasso` in `simulation/src/dstar.cpp:287`**
After `make_planner` runs, `planner.g[s]` holds backward distance from `s` to `s_imag`.
Walk forward greedily: at each state `s`, pick neighbor `nb` minimizing `nb.cost + g[nb.dst]`.
Stop when you land on an accepting state. The cycle is already in `planner.cycle_path[accepting_state]`.
```
reconstruct_lasso(wpa, planner, start):
    current = start,  prefix_ids = []
    loop:
        prefix_ids.push(current)
        if wpa.is_accepting(current): break
        best_cost = +inf,  best_next = INVALID
        for nb in wpa.neighbors_ext(current):
            c = nb.cost + get_g(planner.g, nb.dst)
            if c < best_cost: best_cost = c, best_next = nb.dst
        if best_next == INVALID: return {}
        current = best_next
    cycle_ids = planner.cycle_path[current]
    return LassoResult { to_pos(wpa, prefix_ids), to_pos(wpa, cycle_ids) }
```

**1b. Expose `reconstruct_lasso` — remove `static`, add declaration to `dstar.hpp`**
```cpp
LassoResult reconstruct_lasso(const WPA& wpa, const DStarPlanner& planner, unsigned start);
```

**1c. Update `simulation/apps/sim.cpp:81` — swap A* for D***
```cpp
// replace:  LassoResult lasso = astar_find_path(wpa);
DStarPlanner planner = make_planner(wpa, ReplanMode::DSTAR_INCREMENTAL);
LassoResult lasso = reconstruct_lasso(wpa, planner, wpa.init_state());
```
Verify the path looks correct visually before moving on.

---

### Phase 2 — Detect changes and return changed nodes

**2a. Store `pred_map` in `DStarPlanner` (needed by replan)**
Add field to struct in `dstar.hpp`:
```cpp
std::unordered_map<unsigned, std::vector<unsigned>> pred_map;
```
In `make_planner`, assign `planner.pred_map = pred_map` before returning.

**2b. New function: `detect_changed_states(wpa, changed_cells) -> std::vector<unsigned>`**
Scan all product states; return IDs where `wpa.pos_of(s)` matches any cell in `changed_cells`.

**2c. Implement `dstar_replan` in `dstar.cpp:142` (currently empty)**
Per Algorithm 3 of LTL-D* paper (Ren et al. 2024):
- For `FULL_RECOMPUTE`: call `make_planner` from scratch
- For `DSTAR_INCREMENTAL`:
  1. For each changed state `s`: call `wpa.set_state_exit_weight(s, +inf or 1.0)`
  2. For each predecessor of `s` in `pred_map`: recompute rhs and call `update_vertex`
  3. Call `compute_shortest_path` from current position to `s_imag`
  4. Return `reconstruct_lasso(wpa, planner, current)`
- Also: make `compute_shortest_path` non-static so `dstar_replan` can call it
- Update return type in `dstar.hpp` from `std::vector<unsigned>` to `LassoResult`

**2d. Visualizer callback in `grid_vis.hpp`**
Change `dynamic_visulizer` to accept a callback:
```cpp
void dynamic_visulizer(
    GridWorld& world,
    const std::vector<std::vector<Pos>>& initial_path,
    std::function<LassoResult(Pos, bool)> on_toggle
);
```
`on_toggle(pos, is_now_blocked)` is called on each cell click; returns the new lasso.
In `sim.cpp`, provide a lambda that calls `detect_changed_states` → `dstar_replan`.