#pragma once

#include <vector>
#include "ts.hpp"
#include "types.hpp"
#include "grid_world.hpp"
#include "dstar.hpp"
#include "wpa.hpp"
#include "sensing.hpp"

// takes in full path result
void static_visualizer(
    const GridWorld& world,
    const std::vector<std::vector<Pos>>& path_segments
);

// updates graph to run in real time - something
// void dynamic_visulizer(std::vector<std::vector<Pos>> path );

// world:      ground truth. Click-to-toggle writes Dynamic obstacles here.
// robot_map:  the robot's belief map — what the product/WPA/planner were
//             built from. Only sensing (sense()+reveal(), gated by
//             sensor_cfg) can change it; the robot never sees `world`
//             directly.
// sensor_cfg: non-const — radius is live-adjustable in the GUI ('['/']').
void dynamic_visulizer(
    GridWorld& world,
    GridWorld& robot_map,
    const LassoResult& lasso,
    WPA& wpa,
    DStarPlanner& planner,
    SensorConfig& sensor_cfg
);