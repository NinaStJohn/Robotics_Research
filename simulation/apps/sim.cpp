/*
This is the simulation file
- Run the simulation step by step

features to look into
- interact with map to 'change enviorment'
- speed (adjustable timer)
- run log

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

    // DFS on strongly connected comp?
    // make solution cycle

    // ------------------------------------------------------------------
    // LTL test menu — uncomment ONE `ltl` below. Every atomic proposition
    // used must be placed in the world via world.set_label (see label
    // templates further down). spot syntax: G F X U & | ! ->.
    //
    //   Reachability / coverage
    // std::string ltl = "F a";                          // reach a once          [a]
    // std::string ltl = "F a & F b";                    // reach a and b         [a,b]
    // std::string ltl = "F a & F b & F c";              // reach a,b,c           [a,b,c]
    //
    //   Ordered visit (once, in order)
    // std::string ltl = "F(a & F(b & F c))";            // a then b then c       [a,b,c]
    //
    //   Patrol / surveillance (infinitely often)
    // std::string ltl = "G F a";                        // visit a forever       [a]
    // std::string ltl = "G F a & G F b";                // patrol a,b            [a,b]
    // std::string ltl = "G F a & G F b & G F c";        // patrol a,b,c          [a,b,c]
    //
    //   Response / alternation
    // std::string ltl = "G(a -> F b)";                  // where I started       [a,b]
    // std::string ltl = "G(a -> F b) & G(b -> F a)";    // alternate a,b         [a,b]
    //
    //   Strict ordered alternation (no repeat until the other is seen)
    // std::string ltl = "G(a -> X(!a U b)) & G(b -> X(!b U a))";              //  [a,b]
    //
    //   Avoidance / safety
    // std::string ltl = "F a & G !c";                   // reach a, never c      [a,c]
    // std::string ltl = "G F a & G F b & G !c";         // patrol a,b, avoid c   [a,b,c]
    //
    //   LTL-D* paper benchmark (Ren et al. 2024, §V): repetitively visit
    //   a,b,c,d strictly in order, never skipping ahead. Needs labels a,b,c,d.
    // std::string ltl =
    //   "G(a -> X((!a & !d & !c) U (b & X((!b & !a & !d) U "
    //   "(c & X((!c & !b & !a) U (d & X((!d & !c & !b) U a))))))))";          //  [a,b,c,d]

    GridWorld world(6, 6);
    // Turtlebot bot1({0, 0});                 // start on a
    // world.set_label({0, 0}, "a", true);     // bottom-left
    // world.set_label({5, 0}, "b", true);     // bottom-right
    // world.set_label({5, 5}, "c", true);     // top-right
    // world.set_label({0, 5}, "d", true);     // top-left
    // world.set_blocked({2, 0}, true);
    // world.set_blocked({2, 1}, true);
    // world.set_blocked({2, 2}, true);   // wall up x=2, gap at y=3..5
    std::string ltl = "G F a & G F b & G !e";   // patrol a,b; never enter e


    // ------------------------------------------------------------------

    // LTL formula
    // std::string ltl = "G(a -> Fm) & G(m -> Fa)";

    // GridWorld world(6, 6);
    Turtlebot bot1({0, 0});
    // world.set_label({0,0}, "a", true);
    // // world.set_label({5,0}, "b", true);
    // // extra label placements for the formulas above (uncomment as needed):
    // world.set_label({5,0}, "b", true);
    // world.set_label({0,5}, "c", true);
    // world.set_label({2,2}, "d", true);

    // world.set_blocked({1,1}, true);
    // world.set_blocked({1,2}, true);
    // world.set_blocked({1,3}, true);
    // world.set_blocked({0,3}, true);

    
    // world.set_blocked({1,1}, true);
    // // world.set_blocked({2,2}, true);
    // world.set_blocked({3,3}, true);
    // world.set_blocked({4,4}, true);


    // world.setblocked(1.0);


    world.set_label({0,0}, "a", true);
    world.set_label({5,5}, "b", true);
    world.set_label({2,2}, "e", true);          // forbidden cell on the diagonal

    // LTL graph  --- BUG: INCLUDE THE ROBOT LOCATION IN THE BUILD - change the init!!!!!!!!!!!!
    ProductBundle bundle = build_product_from_world_robot_ltl(world, bot1, ltl);

    // wrap in WPA for weighted search
    WPA wpa(std::move(bundle));

    // DEBUG: dump full product automaton
    std::cout << "=== product states ===\n";
    for (unsigned s = 0; s < wpa.prod()->num_states(); ++s) {
        std::cout << "  state=" << s
                  << " nba=" << wpa.nba_state_of(s)
                  << " pos=(" << wpa.pos_of(s).x << "," << wpa.pos_of(s).y << ")"
                  << " accepting=" << wpa.is_accepting(s) << "\n";
    }
    std::cout << "=== init=" << wpa.init_state() << " ===\n";

    // get lasso (prefix + cycle) using astar
    // LassoResult lasso = astar_find_path(wpa);

    DStarPlanner planner = make_planner(wpa, ReplanMode::DSTAR_INCREMENTAL);
    unsigned start = wpa.state_of(bot1.position(), wpa.nba_init_state());
    LassoResult lasso = dstar_plan(wpa, planner, start);

    std::cout << "Prefix length: " << lasso.prefix.size() << "\n";
    for (const Pos& p : lasso.prefix)
        std::cout << "  (" << p.x << ", " << p.y << ")\n";

    std::cout << "Cycle length: " << lasso.cycle.size() << "\n";
    for (const Pos& p : lasso.cycle)
        std::cout << "  (" << p.x << ", " << p.y << ")\n";

    // grid world vizulizer — prefix in one color, cycle in another
    dynamic_visulizer(world, lasso, wpa, planner);

    return 0;
}




/*
- Fix the TS calculation to include the robot location
- Do the Dstar replanning
- RRT alg paper maybe or maybe not
- adding new nodes psuedo code to product hypotetical
            USE thE NBA
*/