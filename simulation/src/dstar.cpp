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
#include "debug_log.hpp"

// ------------------------------------------------------------------
// Helper types
// ------------------------------------------------------------------

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

// ------------------------------------------------------------------
// Private function declarations
// ------------------------------------------------------------------

// Cycle search
static PathResult astar_cycle_search(const WPA& wpa, unsigned start);

// forward declaration (definition below; signature also in dstar.hpp)
DStarKey calculate_key(
    const DStarPlanner& planner,
    unsigned s, unsigned sstart
);
// forward declaration (definition below; signature also in dstar.hpp)
void compute_shortest_path(
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

double get_g(const std::unordered_map<unsigned, double>& m, unsigned s) {
    std::unordered_map<unsigned, double>::const_iterator it = m.find(s);
    return it == m.end() ? std::numeric_limits<double>::infinity() : it->second;
}

double get_rhs(const std::unordered_map<unsigned, double>& m, unsigned s) {
    std::unordered_map<unsigned, double>::const_iterator it = m.find(s);
    return it == m.end() ? std::numeric_limits<double>::infinity() : it->second;
}

// ------------------------------------------------------------------
// to_pos / to_pos_ids
//
// Converts product state IDs to grid Pos, deduplicating consecutive
// identical positions (caused by NBA transitions with no grid move).
// to_pos_ids returns both in parallel so callers can keep the IDs.
// ------------------------------------------------------------------
static std::vector<Pos> to_pos(const WPA& wpa, const std::vector<unsigned>& ids)
{
    std::vector<Pos> out;
    for (unsigned id : ids) {
        Pos pos = wpa.pos_of(id);
        if (out.empty() || out.back().x != pos.x || out.back().y != pos.y)
            out.push_back(pos);
    }
    return out;
}

static void to_pos_ids(
    const WPA& wpa,
    const std::vector<unsigned>& ids,
    std::vector<Pos>& out_pos,
    std::vector<unsigned>& out_ids
){
    for (unsigned id : ids) {
        Pos pos = wpa.pos_of(id);
        if (out_pos.empty() || out_pos.back().x != pos.x || out_pos.back().y != pos.y) {
            out_pos.push_back(pos);
            out_ids.push_back(id);
        }
    }
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

                // DEBUG: print cycle info
                std::cout << "[DBG] accepting state " << s << " at (" << wpa.pos_of(s).x
                          << "," << wpa.pos_of(s).y << "): cycle length=" << result.path.size()
                          << " cost=" << result.cost << "\n";
                std::cout << "      path: ";
                for (unsigned id : result.path) {
                    std::cout << id << "(" << wpa.pos_of(id).x << "," << wpa.pos_of(id).y << ") ";
                }
                std::cout << "\n";
            } else {
                std::cout << "[DBG] accepting state " << s << " at (" << wpa.pos_of(s).x
                          << "," << wpa.pos_of(s).y << "): NO CYCLE FOUND\n";
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

    // -------------------------------------------------------------
    // DEBUG: structural reachability to the accepting set.
    //
    // Reverse-BFS from every accepting state over pred_map. A state
    // is "can_reach_accept" if some accepting state is reachable by
    // following FORWARD edges (we walk them backwards here). This is
    // purely topological — it ignores edge weights/blocking — so if
    // an nba=2 state shows reach=NO here, the product itself has no
    // path from nba=2 to the accepting set (NBA/product bug), not a
    // weighting/replanning problem.
    // -------------------------------------------------------------
    {
        const unsigned N = wpa.prod()->num_states();
        std::vector<char> can_reach(N, 0);
        std::vector<unsigned> frontier;
        for (unsigned s = 0; s < N; ++s) {
            if (wpa.is_accepting(s)) { can_reach[s] = 1; frontier.push_back(s); }
        }
        while (!frontier.empty()) {
            unsigned v = frontier.back(); frontier.pop_back();
            std::unordered_map<unsigned, std::vector<unsigned>>::const_iterator pit = pred_map.find(v);
            if (pit == pred_map.end()) continue;
            for (unsigned p : pit->second) {
                if (!can_reach[p]) { can_reach[p] = 1; frontier.push_back(p); }
            }
        }

        std::ofstream& log = dbg("reachability.log");
        log << "=== structural reachability to accepting set (topological, ignores weights) ===\n";
        for (unsigned s = 0; s < N; ++s) {
            log << "s=" << s
                << " nba=" << wpa.nba_state_of(s)
                << " pos=(" << wpa.pos_of(s).x << "," << wpa.pos_of(s).y << ")"
                << " accepting=" << wpa.is_accepting(s)
                << " reach_accept=" << (can_reach[s] ? "YES" : "NO")
                << " -> succ:";
            for (const WPA::Neighbor& nb : wpa.neighbors_ext(s)) {
                log << " " << nb.dst << "(nba=" << wpa.nba_state_of(nb.dst)
                    << ",c=" << nb.cost << ")";
            }
            log << "\n";
        }
        log << std::flush;
    }

    // init D* tables
    // g and rhs == inf bc uninit
    planner.rhs[planner.s_imag] = 0;
    planner.km = 0;

    // insert s_imag with U into key (0,0)
    planner.U.push({{0.0, 0.0}, planner.s_imag});

    // Seed accepting states with precomputed cycles as entry points to s_imag.
    // rhs[s] = cycle_cost[s]: from s, one full cycle costs cycle_cost[s], then s_imag is reached.
    // Without this, D* only starts from s_imag which has no real predecessors, so nothing propagates.
    for (unsigned s = 0; s < wpa.prod()->num_states(); ++s) {
        if (wpa.is_accepting(s) && planner.cycle_cost.count(s) > 0) {
            planner.rhs[s] = planner.cycle_cost.at(s);
            planner.U.push({calculate_key(planner, s, init), s});
        }
    }

    // run D* lite
    compute_shortest_path(wpa, planner, pred_map, init, planner.s_imag);

    // save states for recalculation
    planner.pred_map = pred_map;
    planner.slast    = init;
    planner.mode     = mode;
    return planner;
}



// ------------------------------------------------------------------
// D* Lite implementation (based on Koenig & Likhachev 2002, Figure 4)
// ------------------------------------------------------------------

DStarKey calculate_key(
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

void compute_shortest_path(
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
        double rhs_u = get_rhs(planner.rhs, u);

        // {09} lazy deletion: state became consistent after being inserted — skip
        if (g_u == rhs_u) continue;

        // Line 16-21: overconsistent (g > rhs) — propagate improvements
        if (g_u > rhs_u) {
            planner.g[u] = rhs_u;

            // Update predecessors
            std::unordered_map<unsigned, std::vector<unsigned>>::const_iterator pred_it = pred_map.find(u);
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
            std::unordered_map<unsigned, std::vector<unsigned>>::const_iterator pred_it = pred_map.find(u);
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

                // {26-27}: if rhs was built on the old cost, recompute it
                if (get_rhs(planner.rhs, s) == cost_s_to_u + gold) {
                    if (s != sgoal) {
                        double new_rhs = std::numeric_limits<double>::infinity();
                        for (const WPA::Neighbor& nb : wpa.neighbors_ext(s)) {
                            new_rhs = std::min(new_rhs, nb.cost + get_g(planner.g, nb.dst));
                        }
                        if (wpa.is_accepting(s) && planner.cycle_cost.count(s) > 0) {
                            new_rhs = std::min(new_rhs, planner.cycle_cost[s] + get_g(planner.g, sgoal));
                        }
                        planner.rhs[s] = new_rhs;
                    }
                }
                // {28}: UpdateVertex called for ALL predecessors, not just those satisfying {26}
                update_vertex(wpa, planner, s, sstart);
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

LassoResult dstar_plan(
    const WPA& wpa,
    const DStarPlanner& planner,
    unsigned start
){

    unsigned current = start;
    std::vector<unsigned> prefix_ids{};

    std::cout << "[DBG dstar_plan] start=" << start << " at (" << wpa.pos_of(start).x
              << "," << wpa.pos_of(start).y << ")\n";

    while (true){
        prefix_ids.push_back(current);

        // found the end of the prefix
        if (wpa.is_accepting(current)){
            std::cout << "[DBG dstar_plan] found accepting state " << current << " at ("
                      << wpa.pos_of(current).x << "," << wpa.pos_of(current).y << ")\n";
            break;
        }

        // find the best neighnor
        double best_cost = std::numeric_limits<double>::infinity();
        unsigned best_next = wpa.prod() -> num_states(); // sentinel

        // loop: check each neighbor
        for (const WPA::Neighbor& nb : wpa.neighbors_ext(current)){
            // cost to reach this neighbor via its g-value
            double c = nb.cost + get_g(planner.g, nb.dst);

            // check to see what is the best so far
            if (c < best_cost){
                best_cost = c;          // retain cost
                best_next = nb.dst;     // remeber which state to go to
            }
        }

        // after loop: best_next is the neighbor with minimum total cost
        if (best_next == wpa.prod()->num_states()) {
            std::cout << "[DBG dstar_plan] no neighbor found from state " << current
                      << " (" << wpa.pos_of(current).x << "," << wpa.pos_of(current).y << ")\n";
            for (const WPA::Neighbor& nb : wpa.neighbors_ext(current)) {
                std::cout << "  nb=" << nb.dst
                          << " pos=(" << wpa.pos_of(nb.dst).x << "," << wpa.pos_of(nb.dst).y << ")"
                          << " edge_cost=" << nb.cost
                          << " g[nb]=" << get_g(planner.g, nb.dst)
                          << (nb.dst == current ? " [self-loop]" : "") << "\n";
            }
            return {};  // error: no valid neighbor found
        }

        std::cout << "[DBG dstar_plan] step " << current << "(" << wpa.pos_of(current).x
                  << "," << wpa.pos_of(current).y << ") -> " << best_next << "("
                  << wpa.pos_of(best_next).x << "," << wpa.pos_of(best_next).y << ") cost="
                  << best_cost << "\n";

        current = best_next;
    }

    if (start == wpa.init_state() && wpa.is_accepting(start) && planner.cycle_cost.count(start) > 0) {
        // Pick first accepting state that's not the start
        unsigned switched_state = start;
        for (unsigned s = 0; s < wpa.prod()->num_states(); s++) {
            if (wpa.is_accepting(s) && s != start && planner.cycle_cost.count(s) > 0) {
                switched_state = s;
                std::cout << "[DBG dstar_plan] switched from state " << start << " to state " << switched_state << "\n";
                break;
            }
        }
        
        // Use start's cycle as prefix, and switched_state's cycle as the cycle
        if (switched_state != start) {
            std::vector<Pos>      prefix_pos; std::vector<unsigned> prefix_ids_out;
            std::vector<Pos>      cycle_pos;  std::vector<unsigned> cycle_ids_out;
            to_pos_ids(wpa, planner.cycle_path.at(start),          prefix_pos, prefix_ids_out);
            to_pos_ids(wpa, planner.cycle_path.at(switched_state),  cycle_pos,  cycle_ids_out);
            return LassoResult{prefix_pos, cycle_pos, prefix_ids_out, cycle_ids_out};
        }
    }

    // now current is an accepting state
    if (planner.cycle_path.count(current) == 0) {
        std::cout << "[DBG dstar_plan] no cycle precomputed for accepting state " << current << "\n";
        return {};  // no cycle precomputed
    }

    const std::vector<unsigned>& cycle_ids = planner.cycle_path.at(current);
    std::cout << "[DBG dstar_plan] using cycle of length " << cycle_ids.size() << "\n";

    std::vector<Pos>      prefix_pos; std::vector<unsigned> prefix_ids_out;
    std::vector<Pos>      cycle_pos;  std::vector<unsigned> cycle_ids_out;
    to_pos_ids(wpa, prefix_ids, prefix_pos, prefix_ids_out);
    to_pos_ids(wpa, cycle_ids,  cycle_pos,  cycle_ids_out);
    return LassoResult{prefix_pos, cycle_pos, prefix_ids_out, cycle_ids_out};
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
        std::unordered_map<unsigned, double>::iterator it = g_cost.find(cur.state);
        if (it != g_cost.end() && cur.g > it->second)
            continue;

        // real neighbors
        for (const WPA::Neighbor& nb : wpa.neighbors_ext(cur.state)) {
            // cycle mode: intercept edges back to start — redirect to q_imag
            // so parent[start] is never overwritten and reconstruction stays clean
            if (nb.dst == start) {
                double cost_back = cur.g + nb.cost;
                std::unordered_map<unsigned, double>::iterator it3 = g_cost.find(q_imag);
                if (it3 == g_cost.end() || cost_back < it3->second) {
                    g_cost[q_imag] = cost_back;
                    parent[q_imag] = cur.state;
                    open.push({cost_back, cost_back, q_imag});
                }
                continue;
            }

            double tentative_g = cur.g + nb.cost;
            std::unordered_map<unsigned, double>::iterator it2 = g_cost.find(nb.dst);
            if (it2 != g_cost.end() && tentative_g >= it2->second)
                continue;
            g_cost[nb.dst] = tentative_g;
            parent[nb.dst] = cur.state;
            open.push({tentative_g, tentative_g, nb.dst});
        }

        // free edge to q_imag for accepting states
        if (cur.state != start && wpa.is_accepting(cur.state)) {
            std::unordered_map<unsigned, double>::iterator it3 = g_cost.find(q_imag);
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
