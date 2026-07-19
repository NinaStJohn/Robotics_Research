#pragma once

#include <vector>
#include "types.hpp"
#include "grid_world.hpp"
#include "wpa.hpp"
#include "dstar.hpp"   // StateChange

// ------------------------------------------------------------------
// Sensing / partial observability
//
// The robot plans off its own belief map (a GridWorld), never the
// ground-truth world directly. Static structure (walls) is fixed at
// construction (seed_from_static) and never re-sensed — only Dynamic
// cells are discoverable at runtime, via whichever sensor model
// `sense()` implements.
//
// `sense()` is the seam a future raytrace model plugs into: it already
// takes the ground-truth `world` (not just the robot's position), since
// raytrace-style occlusion needs Static geometry to compute line of
// sight. Radius-based sensing (today) just ignores that and does a
// pure distance check.
// ------------------------------------------------------------------

enum class SensorMetric   { Chebyshev, Manhattan, Euclidean };
enum class SensingCadence { EveryTick, OnMove, OnDemand };   // only EveryTick wired up so far

struct SensorConfig {
    double         radius;
    SensorMetric   metric  = SensorMetric::Chebyshev;
    SensingCadence cadence = SensingCadence::EveryTick;
};

// Cells within `cfg.radius` of `center` (by `cfg.metric`), in-bounds.
std::vector<Pos> sense(const SensorConfig& cfg, Pos center, const GridWorld& world);

// Copies Dynamic-cell truth from `world` into `robot_map` for every cell in
// `visible`, marks each visible cell discovered, and returns the cells whose
// belief actually changed (i.e. what the robot didn't already know). Never
// touches Static cells — those are seeded once at construction and are not
// re-sensed here.
std::vector<Pos> reveal(GridWorld& robot_map, const GridWorld& world,
                         const std::vector<Pos>& visible);

// Fresh robot belief map: same bounds + labels as `world`, Static cells
// copied in and marked discovered, everything else Free/undiscovered.
GridWorld seed_from_static(const GridWorld& world);

// Turns a set of touched product states into the planner-ready StateChange
// list dstar_replan expects: for each state, reads its cost BEFORE this
// replan (wpa.state_exit_weight) and derives the cost AFTER from `belief`
// (already updated by reveal()/the caller). States whose cost didn't
// actually change are dropped. Must be called before any of these states'
// weights are mutated.
std::vector<StateChange> build_state_changes(
    const WPA& wpa,
    const GridWorld& belief,
    const std::vector<unsigned>& changed_states
);
