#pragma once

#include <vector>
#include "types.hpp"
#include "wpa.hpp"

// TODO: when lasso (prefix + cycle) support is added, return type should
//       become struct AStarResult { vector<Pos> prefix; vector<Pos> cycle; }
//       so the visualizer and sim can distinguish the two segments.
std::vector<Pos> astar_find_path(const WPA& wpa, const GridWorld& world);