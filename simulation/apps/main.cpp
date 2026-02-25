/*
    Just pass in a run to see if it is valid
    Will show all the NBA, TS and product graphs
    As well has a highlighted run on the graph that 
    is the optimal run from A*
*/
#include "robot.hpp"
#include "grid_world.hpp"
#include "ts.hpp"
#include <filesystem>

// make a 2x2 grid world
// pass in G(a -> Fb)


int main() {
    // make outp0ut
    std::filesystem::create_directory("output");
    GridWorld world(2, 2);
    // world.setblocked(1.0);
    Turtlebot bot1(0, 0);

    // LTL formula 
    std::string ltl = "G(a -> Fb)";

    // LTL graph
    spot::twa_graph_ptr product =
        build_product_from_world_robot_ltl(world, bot1, ltl);

    // astar(product);


    return 0;
}