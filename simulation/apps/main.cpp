/*
    Just pass in a run to see if it is valid
    Will show all the NBA, TS and product graphs
    As well has a highlighted run on the graph that 
    is the optimal run from A*
*/

#include <filesystem>
#include <vector>
#include <spot/tl/parse.hh>
#include <spot/twaalgos/translate.hh>

#include "robot.hpp"
#include "grid_world.hpp"
#include "ts.hpp"
#include "dstar.hpp"
#include "grid_vis.hpp"
#include "wpa.hpp"

// make a 2x2 grid world
// pass in G(a -> Fb)


int main() {
    // make outp0ut
    std::filesystem::create_directory("output");
    // GridWorld world(2, 2);

    // world.set_label({0,0}, "a", true);
    // world.set_label({1,1}, "b", true);
    // world.set_label({0,1}, "c", true);
    // world.set_label({1,0}, "d", true);

    // unhardcode 'b'
    // DFS on strongly connected comp?
    // make solution cycle

    // LTL formula 
    std::string ltl = "G(a -> Fd)";
    
    GridWorld world(3, 3);
    Turtlebot bot1({0,0});
    world.set_label({0,0}, "a", true);
    world.set_label({2,2}, "d", true);

    // world.set_blocked({1,1}, true);
    // world.set_blocked({1,2}, true);
    // world.set_blocked({1,3}, true);
    // world.set_blocked({0,3}, true);

    
    // world.set_blocked({1,1}, true);
    // world.set_blocked({2,2}, true);
    // world.set_blocked({3,3}, true);
    // world.set_blocked({4,4}, true);


    // world.setblocked(1.0);


    // LTL graph
    ProductBundle bundle = build_product_from_world_robot_ltl(world, bot1, ltl);

    // wrap in WPA for weighted search
    WPA wpa(std::move(bundle));

    // for (unsigned s = 0; s < wpa.prod()->num_states(); ++s) {
    //     unsigned nba_state = s % wpa.nba_size();
    //     std::cout << "state " << s << " nba=" << nba_state
    //               << " pos(" << wpa.pos_of(s).x << "," << wpa.pos_of(s).y << ")"
    //               << " accepting: " << wpa.is_accepting(s) << "\n";
    // }

    // unsigned init = wpa.init_state();
    // std::cout << "init state: " << init << "\n";
    // std::cout << "is_accepting(init): " << wpa.is_accepting(init) << "\n";
    // for (const auto& [dst, cost] : wpa.neighbors(init))
    //     std::cout << "  neighbor: " << dst << " cost: " << cost << "\n";

    // for (unsigned s = 0; s < wpa.init_state() + 5; ++s) {
    //     std::cout << "state " << s << " accepting: " << wpa.is_accepting(s) << "\n";
    //     for (const auto& [dst, cost] : wpa.neighbors(s))
    //         std::cout << "  -> " << dst << "\n";
    // }

    // DEBUG: dump NBA structure before product
    // remove after confirming NBA acceptance structure
    spot::twa_graph_ptr nba_debug = spot::translator(spot::make_bdd_dict()).run(
        spot::parse_infix_psl("G(a -> Fb)").f
    );
    std::cout << "NBA states: " << nba_debug->num_states() << "\n";
    for (unsigned s = 0; s < nba_debug->num_states(); ++s) {
        std::cout << "  nba state " << s << "\n";
        for (const auto& e : nba_debug->out(s))
            std::cout << "    -> " << e.dst << " acc=" << e.acc << "\n";
    }

    // get path using astar
    std::vector<Pos> path = astar_find_path(wpa);;

    std::cout << "Path length: " << path.size() << "\n";
    for (const auto& p : path)
        std::cout << "  (" << p.x << ", " << p.y << ")\n";

    // grid world vizulizer
    static_visualizer(world, std::vector<std::vector<Pos>>{path});

    return 0;
}