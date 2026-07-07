/*
    Headless replanning harness — drives dstar_replan WITHOUT the raylib GUI,
    so obstacle toggles can be scripted from the command line instead of clicked.

    Usage:
        ./replantest.out [--suffix=option1|option2] <robot_step> <bx> <by> [<bx2> <by2> ...]

      --suffix=  : which SuffixMode to run replans under (default: option1).
                   option1 = stopgap full A* cycle re-search (recompute_affected_cycles).
                   option2 = paper-faithful incremental SUFFIXREPLAN (one D* Lite
                             search per accepting state, repaired incrementally).
                             Can appear anywhere among the args.
      robot_step : index along the current path the robot sits at when the
                   first obstacle is toggled (0 = path start).
      <bx> <by>  : grid cell to toggle (block if free, unblock if blocked).
                   Multiple cells are applied in sequence, each followed by a
                   full replan, so you can reproduce multi-click scenarios.

    Example:  ./replantest.out 0 2 3                       # block (2,3) with robot at start
              ./replantest.out 6 2 1 2 0                    # at step 6, block (2,1) then (2,0)
              ./replantest.out --suffix=option2 6 2 1 2 0   # same, under Option 2

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

// Pulls "--suffix=option1"/"--suffix=option2" out of args (in place, wherever
// it appears) and returns the SuffixMode it selects, defaulting to OPTION1_ASTAR.
static SuffixMode parse_suffix_mode(std::vector<std::string>& args) {
    SuffixMode mode = SuffixMode::OPTION1_ASTAR;
    for (size_t i = 0; i < args.size(); ) {
        const std::string prefix = "--suffix=";
        if (args[i].rfind(prefix, 0) == 0) {
            std::string value = args[i].substr(prefix.size());
            if (value == "option2") {
                mode = SuffixMode::OPTION2_INCREMENTAL;
            } else if (value == "option1") {
                mode = SuffixMode::OPTION1_ASTAR;
            } else {
                std::cerr << "unknown --suffix value '" << value
                          << "', defaulting to option1\n";
            }
            args.erase(args.begin() + i);
        } else {
            ++i;
        }
    }
    return mode;
}

int main(int argc, char** argv) {
    std::filesystem::create_directory("output");

    std::vector<std::string> args(argv + 1, argv + argc);
    SuffixMode suffix_mode = parse_suffix_mode(args);
    std::cout << "[CLI] suffix_mode = "
              << (suffix_mode == SuffixMode::OPTION2_INCREMENTAL ? "OPTION2_INCREMENTAL" : "OPTION1_ASTAR")
              << "\n";

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

    DStarPlanner planner = make_planner(wpa, ReplanMode::DSTAR_INCREMENTAL, suffix_mode);
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

    // -- parse scripted scenario (args has already had --suffix=... removed) --
    int robot_step = (!args.empty()) ? std::stoi(args[0]) : 0;
    int step_index = std::max(0, std::min(robot_step, (int)path.size() - 1));
    std::cout << "\nrobot placed at step " << step_index
              << " -> state " << path_ids[step_index]
              << " pos=(" << path[step_index].x << "," << path[step_index].y << ")\n";

    // remaining args are (x y) pairs
    for (size_t i = 1; i + 1 < args.size(); i += 2) {
        int bx = std::stoi(args[i]);
        int by = std::stoi(args[i + 1]);
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
