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

// Cycle search.
// true_cycle_only=false: terminate at ANY accepting state (shortest accepting
//   run — used for the legacy prefix search in astar_find_path).
// true_cycle_only=true:  terminate only by returning to `start` (a genuine
//   closed cycle — used by make_planner so cycle_cost is a real lasso suffix).
static PathResult astar_cycle_search(const WPA& wpa, unsigned start,
                                     bool true_cycle_only = false);

// forward declaration (definition below; signature also in dstar.hpp)
DStarKey calculate_key(
    const DStarSearch& search,
    unsigned s, unsigned sstart
);
// forward declaration (definition below; signature also in dstar.hpp)
void compute_shortest_path(
    const WPA& wpa,
    DStarPlanner& planner,
    DStarSearch& search,
    const std::unordered_map<unsigned, std::vector<unsigned>>& pred_map,
    unsigned sstart,
    unsigned sgoal,
    bool drain_all
);
// forward declaration (definition below)
static double edge_cost(
    const WPA& wpa,
    const DStarPlanner& planner,
    const DStarSearch& search,
    unsigned from,
    unsigned to
);
// g-descent from a converged suffix search back to its own goal (s^k_img),
// producing the closed-cycle path in the same format astar_cycle_search
// returns (path starts at the anchor, does NOT repeat it at the end).
static std::vector<unsigned> suffix_cycle_path(
    const WPA& wpa,
    const DStarPlanner& planner,
    const DStarSearch& sfx
);

// ------------------------------------------------------------------
// Helpers
// ------------------------------------------------------------------

// Reserved for the D* Lite heuristic (calculate_key currently uses h=0).
[[maybe_unused]] static double euclidean(Pos a, Pos b) {
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
    ReplanMode mode,
    SuffixMode suffix_mode
){
    DStarPlanner planner;
    unsigned init = wpa.init_state();
    planner.prefix.goal = wpa.prod()->num_states();
    planner.prefix.kind = DStarSearch::SearchKind::PREFIX;

    // build reverse graph (real edges only — this avoids messing with spot
    // internals). Needed before the cycle precompute below: SUFFIXINITIALIZE
    // (Option 2) reads PRED(acc) straight out of this map.
    std::unordered_map<unsigned, std::vector<unsigned>> pred_map;
    for (unsigned s = 0; s < wpa.prod()->num_states(); s++) {
        for (const WPA::Neighbor& nb : wpa.neighbors_ext(s)) {
            pred_map[nb.dst].push_back(s);
        }
    }

    // Cycle precompute: for each accepting state, find its optimal closed
    // cycle (cycle_cost/cycle_path). Two interchangeable strategies —
    // see SuffixMode in dstar.hpp / MIGRATION_NOTES.md.
    if (suffix_mode == SuffixMode::OPTION1_ASTAR) {
        // OPTION 1 (stopgap): one-shot full A* cycle search per accepting state.
        for (unsigned s = 0; s < wpa.prod()->num_states(); s++){
            if (wpa.is_accepting(s)){
                // cycle search from this accepting state — true closed cycle only,
                // so accepting states with no self-cycle (e.g. the init state) get
                // no cycle_cost and are not used as lasso anchors.
                PathResult result = astar_cycle_search(wpa, s, true);
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
    } else {
        // OPTION 2 (Alg. 1 SUFFIXINITIALIZE): one D* Lite search per accepting
        // state, goal = a fresh imaginary node s^k_img. PRED(acc) gets a
        // virtual edge straight to s^k_img (edge_cost() adds it), so running
        // this search to convergence makes g[acc] the cycle cost — the same
        // quantity astar_cycle_search returns, just incrementally repairable
        // later (Step 3 / recompute_affected_cycles) instead of thrown away.
        unsigned next_suffix_goal = wpa.prod()->num_states() + 1; // +1: past prefix's s_imag
        for (unsigned acc = 0; acc < wpa.prod()->num_states(); acc++) {
            if (!wpa.is_accepting(acc)) continue;

            DStarSearch& sfx = planner.suffix[acc];
            sfx.kind            = DStarSearch::SearchKind::SUFFIX;
            sfx.redirect_target = acc;
            sfx.goal             = next_suffix_goal++;

            sfx.pred_map = pred_map;   // real reverse graph, copied per search
            std::unordered_map<unsigned, std::vector<unsigned>>::const_iterator pit = pred_map.find(acc);
            if (pit != pred_map.end()) {
                sfx.pred_map[sfx.goal] = pit->second;   // PRED(acc) -> s^k_img
            }

            sfx.rhs[sfx.goal] = 0;
            sfx.U.push({{0.0, 0.0}, sfx.goal});

            compute_shortest_path(wpa, planner, sfx, sfx.pred_map, acc, sfx.goal, true);

            double cost = get_g(sfx.g, acc);
            if (cost != std::numeric_limits<double>::infinity()) {
                planner.cycle_cost[acc] = cost;
                planner.cycle_path[acc] = suffix_cycle_path(wpa, planner, sfx);

                std::cout << "[DBG suffix2] accepting state " << acc << " at (" << wpa.pos_of(acc).x
                          << "," << wpa.pos_of(acc).y << "): cycle length="
                          << planner.cycle_path[acc].size() << " cost=" << cost << "\n";
            } else {
                std::cout << "[DBG suffix2] accepting state " << acc << " at (" << wpa.pos_of(acc).x
                          << "," << wpa.pos_of(acc).y << "): NO CYCLE FOUND\n";
            }
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

    // Wire the virtual cycle-closing edges (accepting -> s_imag) into the
    // reverse graph. Each accepting state reaches s_imag at cost cycle_cost[s]
    // (see edge_cost). Without this, s_imag has no predecessors and popping it
    // propagates g to nobody, leaving the whole field at infinity.
    for (unsigned s = 0; s < wpa.prod()->num_states(); ++s) {
        if (wpa.is_accepting(s) && planner.cycle_cost.count(s) > 0) {
            pred_map[planner.prefix.goal].push_back(s);
        }
    }

    // init D* tables
    // g and rhs == inf bc uninit
    planner.prefix.rhs[planner.prefix.goal] = 0;
    planner.prefix.km = 0;

    // Seed only the goal s_imag. The accepting states are no longer pre-seeded:
    // their rhs is derived during propagation via the virtual edge above, which
    // keeps a single consistent g-field instead of two disconnected ones.
    planner.prefix.U.push({{0.0, 0.0}, planner.prefix.goal});

    // run D* lite — full build so g is filled for the entire product
    compute_shortest_path(wpa, planner, planner.prefix, pred_map, init, planner.prefix.goal, true);

    // DEBUG: dump the initial g-field so we can confirm it filled the product
    {
        std::ofstream& log = dbg("gfield_initial.log");
        log << "=== g/rhs after initial full build (drain_all) ===\n";
        for (unsigned s = 0; s < wpa.prod()->num_states(); ++s) {
            log << "s=" << s
                << " nba=" << wpa.nba_state_of(s)
                << " pos=(" << wpa.pos_of(s).x << "," << wpa.pos_of(s).y << ")"
                << " accepting=" << wpa.is_accepting(s)
                << " g=" << get_g(planner.prefix.g, s)
                << " rhs=" << get_rhs(planner.prefix.rhs, s)
                << " preds=" << (pred_map.count(s) ? pred_map.at(s).size() : 0)
                << "\n";
        }
        log << "s_imag=" << planner.prefix.goal
            << " g=" << get_g(planner.prefix.g, planner.prefix.goal)
            << " rhs=" << get_rhs(planner.prefix.rhs, planner.prefix.goal)
            << " preds=" << (pred_map.count(planner.prefix.goal) ? pred_map.at(planner.prefix.goal).size() : 0)
            << "\n" << std::flush;
    }

    // save states for recalculation
    planner.prefix.pred_map = pred_map;
    planner.slast       = init;
    planner.mode        = mode;
    planner.suffix_mode = suffix_mode;
    return planner;
}



// ------------------------------------------------------------------
// D* Lite implementation (based on Koenig & Likhachev 2002, Figure 4)
// ------------------------------------------------------------------

DStarKey calculate_key(
    const DStarSearch& search,
    unsigned s,
    [[maybe_unused]] unsigned sstart   // used once h() becomes h(s, sstart)
){
    // key calculation (Figure 4, line 01')

    double h = 0.0;   // TODO - replace with heuristic (euclidean(pos(s), pos(sstart)))
    double min_val = std::min(get_g(search.g, s), get_g(search.rhs, s));
    double key1 = min_val + h + search.km;
    return {key1, min_val};
}

// ------------------------------------------------------------------
// edge_cost
//
// Cost of the directed edge (from -> to), including whichever *virtual*
// goal-closing edge belongs to `search`:
//   PREFIX search: accepting state -> s_imag, cost = cycle_cost[from].
//   SUFFIX search: any real predecessor of search.redirect_target (the
//     anchor this search closes a loop for) -> s^k_img, cost = the real
//     edge cost from `from` to that anchor (Alg. 1 SUFFIXINITIALIZE).
// Both are ADDED edges, not replacements — the real edge (e.g. p->acc)
// stays queryable too, it's just never cheaper than routing through the
// virtual one, since g(goal) == 0 while g(acc) (mid-search) is >= 0.
// Returns +inf when no such edge exists.
// ------------------------------------------------------------------
static double edge_cost(
    const WPA& wpa,
    const DStarPlanner& planner,
    const DStarSearch& search,
    unsigned from,
    unsigned to
){
    if (to == search.goal) {
        if (search.kind == DStarSearch::SearchKind::PREFIX) {
            std::unordered_map<unsigned, double>::const_iterator it = planner.cycle_cost.find(from);
            if (wpa.is_accepting(from) && it != planner.cycle_cost.end())
                return it->second;
            return std::numeric_limits<double>::infinity();
        }
        // SUFFIX: redirected "step back into the anchor" edge.
        for (const WPA::Neighbor& nb : wpa.neighbors_ext(from)) {
            if (nb.dst == search.redirect_target) return nb.cost;
        }
        return std::numeric_limits<double>::infinity();
    }
    for (const WPA::Neighbor& nb : wpa.neighbors_ext(from)) {
        if (nb.dst == to) return nb.cost;
    }
    return std::numeric_limits<double>::infinity();
}

// ------------------------------------------------------------------
// suffix_cycle_path
//
// g-descent from the anchor (sfx.redirect_target) toward sfx.goal, same
// idiom as dstar_plan's prefix descent: at each state, take whichever option
// (a real neighbor, or the virtual closing edge) has the smaller g(dst)+cost.
// The virtual option always represents "stop here, the loop closes via this
// edge" — ties favor closing, since g(goal) == 0 means the virtual option
// can never get cheaper by continuing further.
// ------------------------------------------------------------------
static std::vector<unsigned> suffix_cycle_path(
    const WPA& wpa,
    const DStarPlanner& planner,
    const DStarSearch& sfx
){
    std::vector<unsigned> path;
    unsigned cur = sfx.redirect_target;
    const unsigned sentinel  = wpa.prod()->num_states();
    const unsigned max_steps = sentinel + 1;

    for (unsigned steps = 0; steps < max_steps; ++steps) {
        path.push_back(cur);

        double   best_cost = std::numeric_limits<double>::infinity();
        unsigned best_next  = sentinel;
        for (const WPA::Neighbor& nb : wpa.neighbors_ext(cur)) {
            double c = nb.cost + get_g(sfx.g, nb.dst);
            if (c < best_cost) { best_cost = c; best_next = nb.dst; }
        }

        double virt = edge_cost(wpa, planner, sfx, cur, sfx.goal);
        if (virt <= best_cost) return path;   // loop closes: cur -(virt)-> anchor

        if (best_next == sentinel) return {};  // dead end — g-field says finite but no edge found (shouldn't happen)
        cur = best_next;
    }
    return {};  // safety net: g-field wasn't a genuine finite cycle
}

void compute_shortest_path(
    const WPA& wpa,
    DStarPlanner& planner,
    DStarSearch& search,
    const std::unordered_map<unsigned, std::vector<unsigned>>& pred_map,
    unsigned sstart,
    unsigned sgoal,
    bool drain_all
){
    while (!search.U.empty()) {
        DStarEntry top_entry = search.U.top();
        DStarKey kold = top_entry.first;
        unsigned u = top_entry.second;
        search.U.pop();

        DStarKey knew = calculate_key(search, u, sstart);

        // Line 14-15: lazy deletion — re-insert if key is stale
        if (kold < knew) {
            search.U.push({knew, u});
            continue;
        }

        double g_u = get_g(search.g, u);
        double rhs_u = get_rhs(search.rhs, u);

        // {09} lazy deletion: state became consistent after being inserted — skip
        if (g_u == rhs_u) continue;

        // Line 16-21: overconsistent (g > rhs) — propagate improvements
        if (g_u > rhs_u) {
            search.g[u] = rhs_u;

            // Update predecessors
            std::unordered_map<unsigned, std::vector<unsigned>>::const_iterator pred_it = pred_map.find(u);
            if (pred_it != pred_map.end()) {
                for (unsigned s : pred_it->second) {
                    if (s != sgoal) {
                        // rhs(s) = min(rhs(s), c(s, u) + g(u)).
                        // g(u) was just set to rhs_u above, so use rhs_u here —
                        // NOT the stale g_u (== inf), which would block all
                        // propagation and leave the whole field at infinity.
                        double cost_s_to_u = edge_cost(wpa, planner, search, s, u);
                        double new_rhs = std::min(get_rhs(search.rhs, s), cost_s_to_u + rhs_u);
                        search.rhs[s] = new_rhs;
                    }
                    update_vertex(wpa, search, s, sstart);
                }
            }
        }
        // Line 22-28: underconsistent (g < rhs) — revert and recalculate
        else {
            double gold = g_u;
            search.g[u] = std::numeric_limits<double>::infinity();

            // Update predecessors of u and u itself
            std::unordered_map<unsigned, std::vector<unsigned>>::const_iterator pred_it = pred_map.find(u);
            std::vector<unsigned> affected;
            if (pred_it != pred_map.end()) {
                affected = pred_it->second;
            }
            affected.push_back(u);

            for (unsigned s : affected) {
                // Line 26: if this state's incoming cost from u changed
                double cost_s_to_u = edge_cost(wpa, planner, search, s, u);

                // {26-27}: if rhs was built on the old cost, recompute it
                if (get_rhs(search.rhs, s) == cost_s_to_u + gold) {
                    if (s != sgoal) {
                        double new_rhs = std::numeric_limits<double>::infinity();
                        for (const WPA::Neighbor& nb : wpa.neighbors_ext(s)) {
                            new_rhs = std::min(new_rhs, nb.cost + get_g(search.g, nb.dst));
                        }
                        // virtual goal-closing edge (PREFIX: cycle_cost; SUFFIX:
                        // redirected real edge into the anchor) — see edge_cost().
                        double virt = edge_cost(wpa, planner, search, s, sgoal);
                        if (virt != std::numeric_limits<double>::infinity()) {
                            new_rhs = std::min(new_rhs, virt + get_g(search.g, sgoal));
                        }
                        search.rhs[s] = new_rhs;
                    }
                }
                // {28}: UpdateVertex called for ALL predecessors, not just those satisfying {26}
                update_vertex(wpa, search, s, sstart);
            }
        }

        // Termination. For the initial full build (drain_all) we keep
        // popping until U is empty so g is filled for every state with a
        // path to s_imag. For incremental replanning we stop as soon as
        // sstart is consistent (repair only the affected region).
        if (!drain_all) {
            DStarKey key_start = calculate_key(search, sstart, sstart);
            double g_start = get_g(search.g, sstart);
            double rhs_start = get_rhs(search.rhs, sstart);

            if (search.U.empty()) break;
            DStarKey top_key = search.U.top().first;
            if (top_key >= key_start && g_start == rhs_start) break;
        }
    }
}

// ------------------------------------------------------------------
// recompute_affected_cycles  (OPTION 1 — stopgap suffix repair)
//
// ┌─ MIGRATION NOTE ────────────────────────────────────────────────┐
// │ This is NOT how LTL-D* (Ren et al. 2024) maintains the suffix.   │
// │ The paper gives every accepting state its own D* Lite search     │
// │ (Alg. 1 SUFFIXINITIALIZE) over an imaginary node s^k_img, and    │
// │ repairs it INCREMENTALLY (Alg. 2 SUFFIXREPLAN) when edges change.│
// │ Here we instead re-run a full A* cycle search for any precomputed│
// │ loop a changed cell touches — correct, but it discards the       │
// │ incremental speedup that is the whole point of the paper.        │
// │                                                                   │
// │ To migrate: replace the astar_cycle_search call below with an    │
// │ incremental suffix search (one DStarSearch per accepting state,  │
// │ keyed by `acc`). The COUPLING block below — recompute rhs(acc)   │
// │ from the changed virtual edge <acc, s_imag> and requeue it —     │
// │ stays as-is; that is exactly Alg. 3 lines 14-15. See             │
// │ MIGRATION_NOTES.md.                                              │
// └─────────────────────────────────────────────────────────────────┘
//
// "Touched" test: detect_changed_states returns EVERY product state at a
// changed cell, so a loop crossing that cell necessarily contains one of
// those states — an ID-membership check is exact and complete.
void recompute_affected_cycles(
    const WPA& wpa,
    DStarPlanner& planner,
    const std::vector<unsigned>& changed_states,
    unsigned current
){
    // snapshot the anchors: we mutate cycle_path/cycle_cost while iterating
    std::vector<unsigned> anchors;
    for (std::unordered_map<unsigned, std::vector<unsigned>>::const_iterator it = planner.cycle_path.begin();
         it != planner.cycle_path.end(); ++it) {
        anchors.push_back(it->first);
    }

    for (unsigned acc : anchors) {
        const std::vector<unsigned>& loop = planner.cycle_path.at(acc);

        bool touched = false;
        for (unsigned s : loop) {
            for (unsigned c : changed_states) {
                if (s == c) { touched = true; break; }
            }
            if (touched) break;
        }
        if (!touched) continue;

        double old_cost = planner.cycle_cost.count(acc) > 0
            ? planner.cycle_cost.at(acc)
            : std::numeric_limits<double>::infinity();

        // re-search the true closed cycle on the UPDATED edge weights
        PathResult r = astar_cycle_search(wpa, acc, true);

        double new_cost;
        if (!r.path.empty()) {
            planner.cycle_path[acc] = r.path;
            planner.cycle_cost[acc] = r.cost;
            new_cost = r.cost;
        } else {
            // loop no longer closes -> this accepting state can't anchor a
            // lasso anymore. Drop it; edge_cost() then returns +inf for the
            // virtual edge <acc, s_imag> automatically (it checks cycle_cost).
            planner.cycle_path.erase(acc);
            planner.cycle_cost.erase(acc);
            new_cost = std::numeric_limits<double>::infinity();
        }

        std::cout << "[DBG cycle-repair] anchor " << acc
                  << " cost " << old_cost << " -> " << new_cost
                  << (r.path.empty() ? " (LOOP LOST)" : "") << "\n";

        // COUPLING (Alg. 3, lines 14-15): the virtual edge <acc, s_imag>
        // cost changed, so recompute rhs(acc) over its successors (incl. the
        // virtual edge) and requeue it; the prefix pass then re-ranks anchors.
        if (new_cost != old_cost) {
            double new_rhs = std::numeric_limits<double>::infinity();
            for (const WPA::Neighbor& nb : wpa.neighbors_ext(acc)) {
                new_rhs = std::min(new_rhs, nb.cost + get_g(planner.prefix.g, nb.dst));
            }
            if (planner.cycle_cost.count(acc) > 0) {
                new_rhs = std::min(new_rhs,
                    planner.cycle_cost.at(acc) + get_g(planner.prefix.g, planner.prefix.goal));
            }
            planner.prefix.rhs[acc] = new_rhs;
            update_vertex(wpa, planner.prefix, acc, current);
        }
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

        // End of prefix: an accepting state that actually anchors a cycle.
        // An accepting state with no precomputed cycle (e.g. the init state,
        // whose NBA component is never re-entered) is NOT a valid lasso anchor,
        // so keep descending g past it toward one that is.
        if (wpa.is_accepting(current) && planner.cycle_path.count(current) > 0){
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
            double c = nb.cost + get_g(planner.prefix.g, nb.dst);

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
                          << " g[nb]=" << get_g(planner.prefix.g, nb.dst)
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

    // now current is an accepting state
    if (planner.cycle_path.count(current) == 0) {
        std::cout << "[DBG dstar_plan] no cycle precomputed for accepting state " << current << "\n";
        return {};  // no cycle precomputed
    }

    const std::vector<unsigned>& cycle_ids = planner.cycle_path.at(current);
    std::cout << "[DBG dstar_plan] using cycle of length " << cycle_ids.size() << "\n";

    // print the cycle leg (states + grid cells), as a closed loop -> first
    std::cout << "[DBG dstar_plan] cycle: ";
    for (unsigned id : cycle_ids) {
        std::cout << id << "(" << wpa.pos_of(id).x << "," << wpa.pos_of(id).y << ") ";
    }
    if (!cycle_ids.empty()) {
        std::cout << "-> " << cycle_ids.front()
                  << "(" << wpa.pos_of(cycle_ids.front()).x << ","
                  << wpa.pos_of(cycle_ids.front()).y << ")";
    }
    std::cout << "\n";

    std::vector<Pos>      prefix_pos; std::vector<unsigned> prefix_ids_out;
    std::vector<Pos>      cycle_pos;  std::vector<unsigned> cycle_ids_out;
    to_pos_ids(wpa, prefix_ids, prefix_pos, prefix_ids_out);
    to_pos_ids(wpa, cycle_ids,  cycle_pos,  cycle_ids_out);
    return LassoResult{prefix_pos, cycle_pos, prefix_ids_out, cycle_ids_out};
}

// ------------------------------------------------------------------
// A* cycle search (for pre-computing cycle costs)
// ------------------------------------------------------------------

static PathResult astar_cycle_search(const WPA& wpa, unsigned start,
                                     bool true_cycle_only)
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

        // free edge to q_imag for accepting states.
        // Skipped in true_cycle_only mode: otherwise the search would happily
        // end at a DIFFERENT accepting state and report it as `start`'s cycle
        // (this is exactly what produced the bogus cycle_cost[0] = path-to-(5,5)).
        if (!true_cycle_only && cur.state != start && wpa.is_accepting(cur.state)) {
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

    return LassoResult{ to_pos(wpa, prefix_result.path),
                        to_pos(wpa, cycle_result.path), {}, {} };
}
