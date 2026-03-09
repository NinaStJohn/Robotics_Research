# pragma once

#include <vector>
#include "grid_world.hpp"
#include "ts.hpp"


std::vector<Pos> astar_find_path(
    spot::twa_graph_ptr product, GridWorld world
);