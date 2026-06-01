#pragma once

#include <queue>
#include <unordered_map>
#include <vector>
#include "types.hpp"
#include "wpa.hpp"

struct LassoResult {
    std::vector<Pos> prefix;
    std::vector<Pos> cycle;
};

LassoResult astar_find_path(const WPA& wpa);

enum class ReplanMode { FULL_RECOMPUTE, DSTAR_INCREMENTAL };
    
using DStarKey   = std::pair<double, double>;
using DStarEntry = std::pair<DStarKey, unsigned>;
struct DStarPlanner {
    unsigned                                        s_imag;
    std::unordered_map<unsigned, double>            cycle_cost;   // s_acc -> cost of optimal cycle
    std::unordered_map<unsigned, std::vector<unsigned>>  cycle_path ;  // s_acc -> path IDs
    std::unordered_map<unsigned, double>            g;            // D* cost estimates
    std::unordered_map<unsigned, double>            rhs;          // D* one-step lookahead
    double                                          km;           // D* key modifier
    std::priority_queue<DStarEntry,                     // (lazy deletion, keyed on )
        std::vector<DStarEntry>,                        // Dstar lite stores key and state_id
        std::greater<DStarEntry>> U;
    ReplanMode                                      mode;
};

/*
build D8 tables,
pre-compute cycles,
compute first cycle
*/
DStarPlanner make_planner(
    const WPA& wpa, 
    ReplanMode mode
);

LassoResult dstar_plan(
    const WPA& wpa,
    const DStarPlanner& planner,
    unsigned start
);

// Handle dynamic updates and return new path
LassoResult dstar_replan(
    const WPA& wpa,
    DStarPlanner& planner,
    unsigned current,
    const std::vector<unsigned>& changed_states,
    ReplanMode mode
);
