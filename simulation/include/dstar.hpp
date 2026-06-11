#pragma once

#include <queue>
#include <unordered_map>
#include <vector>
#include "types.hpp"
#include "wpa.hpp"

struct LassoResult {
    std::vector<Pos>      prefix;
    std::vector<Pos>      cycle;
    std::vector<unsigned> prefix_ids;
    std::vector<unsigned> cycle_ids;
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
    std::unordered_map<unsigned, std::vector<unsigned>> pred_map; // reverse graph
    unsigned                                        slast;        // robot pos at last replan
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

// D* Lite engine helpers — non-static so dstar_replan.cpp can call them
double get_g(const std::unordered_map<unsigned, double>& m, unsigned s);
double get_rhs(const std::unordered_map<unsigned, double>& m, unsigned s);

DStarKey calculate_key(
    const DStarPlanner& planner,
    unsigned s,
    unsigned sstart
);

void update_vertex(
    const WPA& wpa,
    DStarPlanner& planner,
    unsigned u,
    unsigned sgoal
);

void compute_shortest_path(
    const WPA& wpa,
    DStarPlanner& planner,
    const std::unordered_map<unsigned, std::vector<unsigned>>& pred_map,
    unsigned sstart,
    unsigned sgoal
);

// Phase 2b: map changed grid cells to product state IDs
std::vector<unsigned> detect_changed_states(
    const WPA& wpa,
    const std::vector<Pos>& changed_cells
);

// Handle dynamic updates and return new path
LassoResult dstar_replan(
    WPA& wpa,
    DStarPlanner& planner,
    unsigned current,
    const std::vector<unsigned>& changed_states,
    bool is_now_blocked,
    ReplanMode mode
);
