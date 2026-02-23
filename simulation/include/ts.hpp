#pragma once
#include <string>
#include <spot/twa/twagraph.hh>

class GridWorld;
class Robot;

spot::twa_graph_ptr build_product_from_world_robot_ltl(const GridWorld& world,
                                                       const Robot& robot,
                                                       const std::string& ltl_infix);