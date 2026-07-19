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

// One product state whose exit cost changed this replan. `cold`/`new_cost`
// are captured PER STATE, before any weights are mutated, so a single
// dstar_replan() call can carry a batch with mixed directions (some states
// newly blocked, others newly cleared) — a radius-based sensor reveal can
// produce exactly that in one call, unlike the old single mouse-click
// trigger which only ever changed one cell in one direction at a time.
struct StateChange {
    unsigned state;
    double   cold;
    double   new_cost;
};

LassoResult astar_find_path(const WPA& wpa);

enum class ReplanMode { FULL_RECOMPUTE, DSTAR_INCREMENTAL };

// Which suffix (cycle) repair strategy to use on replan — lets the two be
// A/B'd on the same scenario instead of only comparable via git history.
// OPTION1_ASTAR:        stopgap — full A* cycle re-search of touched loops
//                       (recompute_affected_cycles). Correct, not incremental,
//                       misses some unblocks (see MIGRATION_NOTES.md).
// OPTION2_INCREMENTAL:  paper-faithful SUFFIXREPLAN (Alg. 1/2) — one D* Lite
//                       search per accepting state, repaired incrementally
//                       (suffixreplan()) instead of fully re-searched.
enum class SuffixMode { OPTION1_ASTAR, OPTION2_INCREMENTAL };

using DStarKey   = std::pair<double, double>;
using DStarEntry = std::pair<DStarKey, unsigned>;

// One D* Lite search table: a goal node (rhs=0), its g/rhs estimates, its
// open queue, key modifier, and the reverse graph it walks backwards over.
// The prefix search (goal = s_imag) is one of these; each accepting state's
// suffix search (goal = s^k_img, Option 2 — MIGRATION_NOTES.md) will be
// another, sharing this same struct and the same engine functions below.
struct DStarSearch {
    // PREFIX: goal = s_imag; the virtual edge INTO goal is <accepting state,
    //   cost=cycle_cost[that state]> (added once the cycle is known).
    // SUFFIX: goal = s^k_img for one specific accepting state (redirect_target);
    //   the virtual edge INTO goal is <p, cost = real edge cost p->redirect_target>
    //   for every real predecessor p of redirect_target — i.e. "step back into
    //   the anchor" is redirected to closing this search's loop instead.
    // Both are ADDED edges alongside the real graph, never replacing a real
    // edge, so edge_cost() only needs a per-`from` special case, not graph
    // surgery. See MIGRATION_NOTES.md Alg. 1 (SUFFIXINITIALIZE).
    enum class SearchKind { PREFIX, SUFFIX };

    SearchKind                                      kind = SearchKind::PREFIX;
    unsigned                                         redirect_target = 0; // SUFFIX only: the anchor (acc)
    unsigned                                        goal;
    std::unordered_map<unsigned, double>            g;            // D* cost estimates
    std::unordered_map<unsigned, double>            rhs;          // D* one-step lookahead
    double                                          km = 0.0;     // D* key modifier
    std::priority_queue<DStarEntry,                     // (lazy deletion, keyed on )
        std::vector<DStarEntry>,                        // Dstar lite stores key and state_id
        std::greater<DStarEntry>> U;
    std::unordered_map<unsigned, std::vector<unsigned>> pred_map; // reverse graph
};

struct DStarPlanner {
    DStarSearch                                     prefix;       // goal = s_imag
    std::unordered_map<unsigned, DStarSearch>        suffix;       // goal = s^k_img, keyed by accepting state (Option 2, not yet populated)
    std::unordered_map<unsigned, double>            cycle_cost;   // s_acc -> cost of optimal cycle
    std::unordered_map<unsigned, std::vector<unsigned>>  cycle_path ;  // s_acc -> path IDs
    ReplanMode                                      mode;
    SuffixMode                                      suffix_mode;  // OPTION1_ASTAR or OPTION2_INCREMENTAL
    unsigned                                        slast;        // robot pos at last replan
};

/*
build D8 tables,
pre-compute cycles,
compute first cycle
*/
DStarPlanner make_planner(
    const WPA& wpa,
    ReplanMode mode,
    SuffixMode suffix_mode = SuffixMode::OPTION1_ASTAR
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
    const DStarSearch& search,
    unsigned s,
    unsigned sstart
);

void update_vertex(
    const WPA& wpa,
    DStarSearch& search,
    unsigned u,
    unsigned sgoal
);

// drain_all=true  -> expand the whole field until U is empty (initial build:
//                    fills g for every state with a path to s_imag).
// drain_all=false -> stop once sstart is consistent (incremental replan repair).
// `planner` is still needed here (rather than just `search`) because edge_cost()
// reads planner.cycle_cost for the PREFIX virtual edge; `search` supplies its
// own goal/kind/redirect_target so the same engine runs prefix or any suffix
// search unchanged.
void compute_shortest_path(
    const WPA& wpa,
    DStarPlanner& planner,
    DStarSearch& search,
    const std::unordered_map<unsigned, std::vector<unsigned>>& pred_map,
    unsigned sstart,
    unsigned sgoal,
    bool drain_all = false
);

// Phase 2b: map changed grid cells to product state IDs
std::vector<unsigned> detect_changed_states(
    const WPA& wpa,
    const std::vector<Pos>& changed_cells
);

// OPTION 1 (stopgap) suffix repair. Re-search, with a full A* cycle search,
// any precomputed loop that a changed product state touches, refresh
// cycle_cost/cycle_path, and couple the new loop cost back into the prefix
// g-field. See MIGRATION_NOTES.md — LTL-D* keeps this incremental instead.
// Only needs each state's id (the "touched" test doesn't care which
// direction it changed — it re-searches from scratch either way).
void recompute_affected_cycles(
    const WPA& wpa,
    DStarPlanner& planner,
    const std::vector<StateChange>& changed_states,
    unsigned current
);

// Shared {43-46}-style rhs repair for one state `u` whose edge cost just
// changed from `cold` to `new_cost`, against a single search's own g/rhs
// (works for the prefix search or any suffix search — the virtual
// goal-closing edge is resolved generically via edge_cost() inside).
// Exposed (not static) so both dstar_replan.cpp's prefix loop and
// suffixreplan()'s per-accepting-state loop share one implementation.
void repair_rhs_for_changed_edge(
    const WPA& wpa,
    const DStarPlanner& planner,
    DStarSearch& search,
    unsigned u,
    double cold,
    double new_cost
);

// OPTION 2 (paper-faithful) suffix repair — Alg. 2 SUFFIXREPLAN. For every
// accepting state's own suffix search: apply the already-updated real edge
// costs (repair_rhs_for_changed_edge), repair incrementally
// (compute_shortest_path, drain_all=false, sstart=acc), read back
// cycle_cost/cycle_path, then run the same COUPLING step as
// recompute_affected_cycles (Alg. 3 lines 14-15). Each state's own
// cold/new_cost travels with it in `changed_states`, so a mixed-direction
// batch (some states newly blocked, others newly cleared) is repaired
// correctly in one call instead of assuming one shared direction.
void suffixreplan(
    const WPA& wpa,
    DStarPlanner& planner,
    const std::vector<StateChange>& changed_states,
    unsigned current
);

// Handle dynamic updates and return new path. `changed_states` carries each
// state's own before/after cost (see StateChange) — built by
// sensing.hpp's build_state_changes() from whatever triggered this replan
// (a sensor reveal, a scripted test toggle, etc.), so the caller decides
// per-state direction rather than dstar_replan assuming one for the whole
// batch.
LassoResult dstar_replan(
    WPA& wpa,
    DStarPlanner& planner,
    unsigned current,
    const std::vector<StateChange>& changed_states,
    ReplanMode mode
);
