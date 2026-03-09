#pragma once

#include <vector>
#include "ts.hpp"
#include "types.hpp"
#include "grid_world.hpp"

// takes in full path result
void static_visualizer(
    const GridWorld& world,
    const std::vector<std::vector<Pos>>& path_segments
);

// updates graph to run in real time - something
// void dynamic_visulizer(std::vector<std::vector<Pos>> path );