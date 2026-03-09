/*
    Just pass in a run to see if it is valid
    Will show all the NBA, TS and product graphs
    As well has a highlighted run on the graph that 
    is the optimal run from A*
*/

#include <filesystem>
#include <vector>

#include "robot.hpp"
#include "grid_world.hpp"
#include "ts.hpp"
#include "dstar.hpp"
#include "grid_vis.hpp"

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

    GridWorld world(5, 5);
    world.set_label({0,0}, "a", true);
    world.set_label({4,2}, "b", true);

    // world.setblocked(1.0);
    Turtlebot bot1({0,0});

    // LTL formula 
    std::string ltl = "G(a -> Fb)";

    // LTL graph
    spot::twa_graph_ptr product =
        build_product_from_world_robot_ltl(world, bot1, ltl);
    
    // get path using astar
    // std::vector<std::vector<Pos>>{path}
    std::vector<Pos> path = astar_find_path(product, world);

    // grid world vizulizer
    static_visualizer(world, std::vector<std::vector<Pos>>{path});

    return 0;
}