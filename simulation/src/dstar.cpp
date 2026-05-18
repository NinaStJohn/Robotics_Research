/*
    D* / A* search over the product automaton.
    astar_find_path returns a LassoResult (prefix + cycle) representing
    the infinite accepting run required by LTL semantics.
*/

#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>

#include "dstar.hpp"

// ------------------------------------------------------------------
// Private function declarations
// ------------------------------------------------------------------

// D* Lite helpers
static DStarKey calculate_key(
    const DStarPlanner& planner,
    unsigned s, unsigned sstart
);
static void update_vertex(
    const WPA& wpa,
    DStarPlanner& planner,
    unsigned u,
    unsigned sgoal
);
static void compute_shortest_path(
    const WPA& wpa,
    DStarPlanner& planner,
    const std::unordered_map<unsigned, std::vector<unsigned>>& pred_map,
    unsigned sstart,
    unsigned sgoal
);

// ------------------------------------------------------------------
// Helpers
// ------------------------------------------------------------------

static double euclidean(Pos a, Pos b) {
    double dx = static_cast<double>(a.x - b.x);
    double dy = static_cast<double>(a.y - b.y);
    return std::sqrt(dx*dx + dy*dy);
}

struct Node {
    double   f;
    double   g;
    unsigned state;
    bool operator>(const Node& o) const { return f > o.f; }
};

struct PathResult {
    std::vector<unsigned> path;
    double                cost;
};

static double get_g(const std::unordered_map<unsigned, double>& m, unsigned s) {
    std::unordered_map<unsigned, double>::const_iterator it = m.find(s);
    return it == m.end() ? std::numeric_limits<double>::infinity() : it->second;
}

static double get_rhs(const std::unordered_map<unsigned, double>& m, unsigned s) {
    std::unordered_map<unsigned, double>::const_iterator it = m.find(s);
    return it == m.end() ? std::numeric_limits<double>::infinity() : it->second;
}

// ------------------------------------------------------------------
// to_pos
//
// Converts product state IDs to grid Pos, deduplicating consecutive
// identical positions (caused by NBA transitions with no grid move).
// ------------------------------------------------------------------
static std::vector<Pos> to_pos(const WPA& wpa, const std::vector<unsigned>& ids)
{
    std::vector<Pos> raw;
    for (unsigned id : ids)
        raw.push_back(wpa.pos_of(id));

    std::vector<Pos> deduped;
    for (const Pos& pos : raw) {
        if (deduped.empty() ||
            deduped.back().x != pos.x ||
            deduped.back().y != pos.y)
            deduped.push_back(pos);
    }
    return deduped;
}

// ------------------------------------------------------------------
// DStar planner
//
// update the product graph (and ts?) to reflect changes
// ------------------------------------------------------------------

DStarPlanner make_planner(
    const WPA& wpa,
    ReplanMode mode
){
    DStarPlanner planner;
    unsigned init = wpa.init_state();
    planner.s_imag = wpa.prod()->num_states();

    // loop over all product states
    for (unsigned s = 0; s < wpa.prod()->num_states(); s++){
        if (wpa.is_accepting(s)){
            // cycle search from this accepting state
            PathResult result = astar_cycle_search(wpa, s);
            if (!result.path.empty()) {
                planner.cycle_path[s] = result.path;
                planner.cycle_cost[s] = result.cost;
            }
        }
    }

    // build reverse graph
    // this avoids messing with spot internals
    std::unordered_map<unsigned, std::vector<unsigned>> pred_map;
    for (unsigned s = 0; s < wpa.prod()->num_states(); s++) {
        for (const WPA::Neighbor& nb : wpa.neighbors_ext(s)) {
            pred_map[nb.dst].push_back(s);
        }
    }

    // init D* tables
    // g and rhs == inf bc uninit
    planner.rhs[planner.s_imag] = 0;
    planner.km = 0;

    // insert s_imag with U into key (0,0)
    planner.U.push({{0.0, 0.0}, planner.s_imag});

    // run D* lite
    compute_shortest_path(wpa, planner, pred_map, init, planner.s_imag);

    return planner;
}

std::vector<unsigned> dstar_replan(
    const WPA& wpa,
    unsigned current,
    std::vector<unsigned> cycle,
    unsigned blockage,
    ReplanMode mode
){
    // FULL_RECOMPUTE: call make_planner again from scratch
    // DSTAR_INCREMENTAL: call wpa.set_state_exit_weight(blockage, infinity),
    // call UpdateVertex on all predecessors of blockage, re-run ComputeShortestPath with updated km
}

// ------------------------------------------------------------------
// D* Lite implementation (based on Koenig & Likhachev 2002, Figure 4)
// ------------------------------------------------------------------

static DStarKey calculate_key(
    const DStarPlanner& planner,
    unsigned s,
    unsigned sstart
){
    // key calculation (Figure 4, line 01')

    double h = 0.0;   // TODO - replace with heuristic (eclusian)
    double min_val = std::min(get_g(planner.g, s), get_g(planner.rhs, s));
    double key1 = min_val + h + planner.km;
    return {key1, min_val};
}

static void update_vertex(
    const WPA& wpa,
    DStarPlanner& planner,
    unsigned u,
    unsigned sgoal
){
    double g_u = get_g(planner.g, u);
    double rhs_u = get_g(planner.rhs, u);

    // c++ does not haeve Update or Remove, so we use lazy deletion
    // Always push with new key (duplicates are OK) & skip stale entries when popping
    if (g_u != rhs_u) {
        planner.U.push({calculate_key(planner, u, sgoal), u});
    }
}

static void compute_shortest_path(
    const WPA& wpa,
    DStarPlanner& planner,
    const std::unordered_map<unsigned, std::vector<unsigned>>& pred_map,
    unsigned sstart,
    unsigned sgoal
){
    while (!planner.U.empty()) {
        DStarEntry top_entry = planner.U.top();
        DStarKey kold = top_entry.first;
        unsigned u = top_entry.second;
        planner.U.pop();

        DStarKey knew = calculate_key(planner, u, sstart);

        // Line 14-15: lazy deletion — re-insert if key is stale
        if (kold < knew) {
            planner.U.push({knew, u});
            continue;
        }

        double g_u = get_g(planner.g, u);
        double rhs_u = get_g(planner.rhs, u);

        // Line 16-21: overconsistent (g > rhs) — propagate improvements
        if (g_u > rhs_u) {
            planner.g[u] = rhs_u;

            // Update predecessors
            auto pred_it = pred_map.find(u);
            if (pred_it != pred_map.end()) {
                for (unsigned s : pred_it->second) {
                    if (s != sgoal) {
                        // rhs(s) = min(rhs(s), c(s, u) + g(u))
                        double cost_s_to_u = std::numeric_limits<double>::infinity();
                        for (const WPA::Neighbor& nb : wpa.neighbors_ext(s)) {
                            if (nb.dst == u) {
                                cost_s_to_u = nb.cost;
                                break;
                            }
                        }
                        double new_rhs = std::min(get_rhs(planner.rhs, s), cost_s_to_u + g_u);
                        planner.rhs[s] = new_rhs;
                    }
                    update_vertex(wpa, planner, s, sstart);
                }
            }
        }
        // Line 22-28: underconsistent (g < rhs) — revert and recalculate
        else {
            double gold = g_u;
            planner.g[u] = std::numeric_limits<double>::infinity();

            // Update predecessors of u and u itself
            auto pred_it = pred_map.find(u);
            std::vector<unsigned> affected;
            if (pred_it != pred_map.end()) {
                affected = pred_it->second;
            }
            affected.push_back(u);

            for (unsigned s : affected) {
                // Line 26: if this state's incoming cost from u changed
                double cost_s_to_u = std::numeric_limits<double>::infinity();
                for (const WPA::Neighbor& nb : wpa.neighbors_ext(s)) {
                    if (nb.dst == u) {
                        cost_s_to_u = nb.cost;
                        break;
                    }
                }

                if (get_rhs(planner.rhs, s) == cost_s_to_u + gold) {
                    // Line 27: recompute rhs(s) from all successors
                    if (s != sgoal) {
                        double new_rhs = std::numeric_limits<double>::infinity();
                        for (const WPA::Neighbor& nb : wpa.neighbors_ext(s)) {
                            new_rhs = std::min(new_rhs, nb.cost + get_g(planner.g, nb.dst));
                        }
                        // Also consider weighted edge to sgoal if s is accepting
                        if (wpa.is_accepting(s) && planner.cycle_cost.count(s) > 0) {
                            new_rhs = std::min(new_rhs, planner.cycle_cost[s] + get_g(planner.g, sgoal));
                        }
                        planner.rhs[s] = new_rhs;
                    }
                    update_vertex(wpa, planner, s, sstart);
                }
            }
        }

        // Check termination condition
        DStarKey key_start = calculate_key(planner, sstart, sstart);
        double g_start = get_g(planner.g, sstart);
        double rhs_start = get_rhs(planner.rhs, sstart);

        if (planner.U.empty()) break;
        DStarKey top_key = planner.U.top().first;
        if (top_key >= key_start && g_start == rhs_start) break;
    }
}

static LassoResult reconstruct_lasso(
    const WPA& wpa,
    const DStarPlanner& planner,
    unsigned start
){
    // TODO: reconstruct lasso from D* tables
    return {};
}

// ------------------------------------------------------------------
// A* cycle search (for pre-computing cycle costs)
// ------------------------------------------------------------------

static PathResult astar_cycle_search(const WPA& wpa, unsigned start)
{
    unsigned q_imag = wpa.prod()->num_states();

    std::unordered_map<unsigned, double>   g_cost;
    std::unordered_map<unsigned, unsigned> parent;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

    g_cost[start] = 0.0;
    open.push({0.0, 0.0, start});

    while (!open.empty()) {
        Node cur = open.top(); open.pop();

        // reached imaginary node — reconstruct path back to start
        if (cur.state == q_imag) {
            std::vector<unsigned> path;
            unsigned s = parent.at(q_imag);  // skip q_imag itself
            while (true) {
                path.push_back(s);
                if (s == start) break;
                s = parent.at(s);
            }
            std::reverse(path.begin(), path.end());
            return {path, cur.g};
        }

        // stale entry check
        auto it = g_cost.find(cur.state);
        if (it != g_cost.end() && cur.g > it->second)
            continue;

        // real neighbors
        for (const WPA::Neighbor& nb : wpa.neighbors_ext(cur.state)) {
            // cycle mode: intercept edges back to start — redirect to q_imag
            // so parent[start] is never overwritten and reconstruction stays clean
            if (nb.dst == start) {
                double cost_back = cur.g + nb.cost;
                auto it3 = g_cost.find(q_imag);
                if (it3 == g_cost.end() || cost_back < it3->second) {
                    g_cost[q_imag] = cost_back;
                    parent[q_imag] = cur.state;
                    open.push({cost_back, cost_back, q_imag});
                }
                continue;
            }

            double tentative_g = cur.g + nb.cost;
            auto it2 = g_cost.find(nb.dst);
            if (it2 != g_cost.end() && tentative_g >= it2->second)
                continue;
            g_cost[nb.dst] = tentative_g;
            parent[nb.dst] = cur.state;
            open.push({tentative_g, tentative_g, nb.dst});
        }

        // free edge to q_imag for accepting states
        if (cur.state != start && wpa.is_accepting(cur.state)) {
            auto it3 = g_cost.find(q_imag);
            if (it3 == g_cost.end() || cur.g < it3->second) {
                g_cost[q_imag] = cur.g;
                parent[q_imag] = cur.state;
                open.push({cur.g, cur.g, q_imag});
            }
        }
    }

    return {};
}

// ------------------------------------------------------------------
// astar_find_path (legacy interface)
//
// Entry point. Runs astar_cycle_search twice:
//   1. prefix: init_state -> first accepting state
//   2. cycle:  that accepting state -> back to itself
// ------------------------------------------------------------------

LassoResult astar_find_path(const WPA& wpa)
{
    unsigned init = wpa.init_state();

    PathResult prefix_result = astar_cycle_search(wpa, init);
    if (prefix_result.path.empty()) {
        std::cerr << "astar_find_path(): no accepting state reachable from init\n";
        return {};
    }

    std::cout << "[DBG] prefix state IDs: ";
    for (unsigned id : prefix_result.path) std::cout << id << " ";
    std::cout << "\n";

    unsigned q_acc = prefix_result.path.back();
    std::cout << "[DBG] cycle search starts from state=" << q_acc
              << " nba=" << wpa.nba_state_of(q_acc)
              << " pos=(" << wpa.pos_of(q_acc).x << "," << wpa.pos_of(q_acc).y << ")\n";

    PathResult cycle_result = astar_cycle_search(wpa, q_acc);
    if (cycle_result.path.empty()) {
        std::cerr << "astar_find_path(): no cycle found from accepting state\n";
        return {};
    }

    std::cout << "[DBG] cycle state IDs: ";
    for (unsigned id : cycle_result.path) std::cout << id << " ";
    std::cout << "\n";

    return { to_pos(wpa, prefix_result.path), to_pos(wpa, cycle_result.path) };
}
