#pragma once
#include <string>
#include <spot/twa/twagraph.hh>

#include <algorithm>
#include <unordered_map>

#include "types.hpp"
#include "robot.hpp"
#include "grid_world.hpp"

class GridWorld;
 
// ------------------------------------------------------------------
// ProductBundle
//   Carries everything downstream (A*, sim, multi-robot) will need.
//   ts_state_to_pos: TS state id -> grid Pos  (for heuristic + vis)
//   act_ap:          Robot::Action -> BDD AP id (for heterogeneous costs later)
//   world_ap:        label name   -> BDD AP id (for weight lookup later)
// ------------------------------------------------------------------
struct ProductBundle {
    spot::twa_graph_ptr                     prod;
    unsigned                                nba_size;
    // After product construction these are keyed by PRODUCT state (not TS state).
    std::unordered_map<unsigned, Pos>       ts_state_to_pos;    // product_state -> grid Pos
    std::unordered_map<unsigned, unsigned>  prod_state_to_nba;  // product_state -> nba_state
    // Reverse lookup: (grid Pos, nba_state) -> product_state. Used by replanning
    // to find the product state for any (robot_position, nba_q) without rebuilding.
    std::unordered_map<std::pair<Pos, unsigned>, unsigned, PosNbaHash> pos_nba_to_prod;
    std::unordered_map<Robot::Action, int>    act_ap;
    std::unordered_map<Robot::Action, double> action_costs;  // per-robot move costs for WPA edge weights
    std::unordered_map<std::string, int>      world_ap;
};
 
ProductBundle build_product_from_world_robot_ltl(
    const GridWorld&   world,
    const Robot&       robot,
    const std::string& ltl_infix
);
 