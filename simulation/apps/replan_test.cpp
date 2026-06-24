/*
    Headless replanning harness — drives dstar_replan WITHOUT the raylib GUI,
    so obstacle toggles can be scripted from the command line instead of clicked.

    Usage:
        ./replantest.out <robot_step> <bx> <by> [<bx2> <by2> ...]

      robot_step : index along the current path the robot sits at when the
                   first obstacle is toggled (0 = path start).
      <bx> <by>  : grid cell to toggle (block if free, unblock if blocked).
                   Multiple cells are applied in sequence, each followed by a
                   full replan, so you can reproduce multi-click scenarios.

    Example:  ./replantest.out 0 2 3        # block (2,3) with robot at start
              ./replantest.out 6 2 1 2 0    # at step 6, block (2,1) then (2,0)

    Mirrors the world setup in apps/sim.cpp. Same logs are written to output/.
*/

#include <filesystem>
#include <iostream>
#include <vector>
#include <string>

#include "robot.hpp"
#include "grid_world.hpp"
#include "ts.hpp"
#include "dstar.hpp"
#include "wpa.hpp"

// Flatten a lasso into a parallel (pos, id) path, dropping consecutive
// duplicate positions — same rule build_flat_path uses in grid_vis.cpp.
static void flatten(
    const LassoResult& lasso,
    std::vector<Pos>& out_pos,
    std::vector<unsigned>& out_ids
){
    out_pos.clear();
    out_ids.clear();
    for (size_t i = 0; i < lasso.prefix.size(); ++i) {
        const Pos& p = lasso.prefix[i];
        if (!out_pos.empty() && out_pos.back().x == p.x && out_pos.back().y == p.y) continue;
        out_pos.push_back(p);
        out_ids.push_back(lasso.prefix_ids[i]);
    }
    for (size_t i = 0; i < lasso.cycle.size(); ++i) {
        const Pos& p = lasso.cycle[i];
        if (!out_pos.empty() && out_pos.back().x == p.x && out_pos.back().y == p.y) continue;
        out_pos.push_back(p);
        out_ids.push_back(lasso.cycle_ids[i]);
    }
}

static void print_path(
    const std::string& tag,
    const std::vector<Pos>& pos,
    int cycle_start
){
    std::cout << tag << " (" << pos.size() << " cells, cycle starts at index "
              << cycle_start << "):\n  ";
    for (size_t i = 0; i < pos.size(); ++i) {
        if ((int)i == cycle_start) std::cout << "| ";   // prefix | cycle divider
        std::cout << "(" << pos[i].x << "," << pos[i].y << ") ";
    }
    std::cout << "\n";

    // cycle leg on its own line: the repeating suffix loop the robot follows
    // forever, written as a closed loop (…last -> first).
    std::cout << "  CYCLE LEG (" << (int)pos.size() - cycle_start << " cells): ";
    for (size_t i = cycle_start; i < pos.size(); ++i)
        std::cout << "(" << pos[i].x << "," << pos[i].y << ") ";
    if (cycle_start < (int)pos.size())
        std::cout << "-> (" << pos[cycle_start].x << "," << pos[cycle_start].y << ")";
    std::cout << "\n";
}

int main(int argc, char** argv) {
    std::filesystem::create_directory("output");

    // -- identical world to apps/sim.cpp --
    std::string ltl = "G(a -> Fb) & G(b -> Fa)";
    GridWorld world(6, 6);
    Turtlebot bot1({0,0});
    world.set_label({0,0}, "a", true);
    world.set_label({5,5}, "b", true);
    world.set_blocked({1,1}, true);
    world.set_blocked({1,2}, true);
    world.set_blocked({1,3}, true);
    world.set_blocked({0,3}, true);
    world.set_blocked({2,2}, true);
    world.set_blocked({3,3}, true);
    world.set_blocked({4,4}, true);

    ProductBundle bundle = build_product_from_world_robot_ltl(world, bot1, ltl);
    WPA wpa(std::move(bundle));

    DStarPlanner planner = make_planner(wpa, ReplanMode::DSTAR_INCREMENTAL);
    unsigned start = wpa.state_of(bot1.position(), wpa.nba_init_state());
    LassoResult lasso = dstar_plan(wpa, planner, start);

    std::vector<Pos>      path;
    std::vector<unsigned> path_ids;
    flatten(lasso, path, path_ids);
    int cycle_start = (int)0;
    {
        // loop restart index = anchor index = deduped prefix length - 1
        // (prefix end and cycle start share the anchor; flatten dedups it).
        std::vector<Pos> ppos; std::vector<unsigned> pids;
        LassoResult pre{lasso.prefix, {}, lasso.prefix_ids, {}};
        flatten(pre, ppos, pids);
        cycle_start = (int)ppos.size() > 0 ? (int)ppos.size() - 1 : 0;
    }

    if (path.empty()) {
        std::cout << "INITIAL PLAN FAILED (empty lasso)\n";
        return 1;
    }
    print_path("INITIAL PATH", path, cycle_start);

    // -- parse scripted scenario --
    int robot_step = (argc > 1) ? std::stoi(argv[1]) : 0;
    int step_index = std::max(0, std::min(robot_step, (int)path.size() - 1));
    std::cout << "\nrobot placed at step " << step_index
              << " -> state " << path_ids[step_index]
              << " pos=(" << path[step_index].x << "," << path[step_index].y << ")\n";

    // remaining argv are (x y) pairs
    for (int i = 2; i + 1 < argc; i += 2) {
        int bx = std::stoi(argv[i]);
        int by = std::stoi(argv[i + 1]);
        Pos cell{bx, by};

        if (!world.in_bounds(cell)) {
            std::cout << "\n(" << bx << "," << by << ") out of bounds, skipping\n";
            continue;
        }

        world.set_blocked(cell, !world.is_blocked(cell));
        bool is_now_blocked = world.is_blocked(cell);
        std::vector<unsigned> changed = detect_changed_states(wpa, {cell});
        unsigned current_state = path_ids[step_index];

        std::cout << "\n=== toggle (" << bx << "," << by << ") -> "
                  << (is_now_blocked ? "BLOCKED" : "FREE")
                  << "  | robot at state " << current_state
                  << " (" << path[step_index].x << "," << path[step_index].y << ")"
                  << " | " << changed.size() << " product state(s) affected ===\n";

        LassoResult new_lasso = dstar_replan(
            wpa, planner, current_state, changed, is_now_blocked, planner.mode);

        if (new_lasso.prefix.empty() && new_lasso.cycle.empty()) {
            std::cout << "REPLAN FAILED (empty lasso)\n";
            continue;
        }

        flatten(new_lasso, path, path_ids);
        std::vector<Pos> ppos; std::vector<unsigned> pids;
        LassoResult pre{new_lasso.prefix, {}, new_lasso.prefix_ids, {}};
        flatten(pre, ppos, pids);
        // loop restart index = anchor index = deduped prefix length - 1
        cycle_start = (int)ppos.size() > 0 ? (int)ppos.size() - 1 : 0;

        step_index = std::min(step_index, (int)path.size() - 1);
        print_path("REPLANNED PATH", path, cycle_start);
    }

    return 0;
}
