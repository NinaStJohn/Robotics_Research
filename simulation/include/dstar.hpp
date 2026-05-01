#pragma once

#include <vector>
#include "types.hpp"
#include "wpa.hpp"

struct LassoResult {
    std::vector<Pos> prefix;
    std::vector<Pos> cycle;
};

LassoResult astar_find_path(const WPA& wpa);
