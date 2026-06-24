/*
    Replanning layer for D* Lite over the product automaton.
    detect_changed_states: maps toggled grid cells -> product state IDs
    dstar_replan:          incremental or full-recompute replan (Phase 2c)
*/

#include <vector>
#include <limits>
#include <iostream>

#include "dstar.hpp"
#include "wpa.hpp"
#include "debug_log.hpp"

// ------------------------------------------------------------------
// detect_changed_states
//
// For each product state, check whether its grid position matches
// any cell in changed_cells. Returns the matching state IDs.
// ------------------------------------------------------------------

std::vector<unsigned> detect_changed_states(
    const WPA& wpa,
    const std::vector<Pos>& changed_cells
){
    std::vector<unsigned> result;

    for (unsigned s = 0; s < wpa.prod()->num_states(); s++) {
        Pos pos = wpa.pos_of(s);
        for (const Pos& cell : changed_cells) {
            if (pos.x == cell.x && pos.y == cell.y) {
                std::cout << "[DBG detect] cell (" << cell.x << "," << cell.y
                          << ") -> product state " << s
                          << " (nba=" << wpa.nba_state_of(s) << ")"
                          << " pos=(" << pos.x << "," << pos.y << ")\n";
                result.push_back(s);
                break;
            }
        }
    }

    std::cout << "[DBG detect] " << changed_cells.size() << " cell(s) changed -> "
              << result.size() << " product state(s) affected\n";

    return result;
}

// ------------------------------------------------------------------
// update_vertex  (Figure 4, lines 07-09)
//
// {07} if g(u) != rhs(u) AND u in U  -> U.Update(u, CalculateKey(u))
// {08} if g(u) != rhs(u) AND u not in U -> U.Insert(u, CalculateKey(u))
// {09} if g(u) == rhs(u) AND u in U  -> U.Remove(u)
//
// Using lazy deletion: cases 07+08 collapse to one push (stale entries
// are skipped on pop). Case 09 is handled by compute_shortest_path
// skipping consistent states when popped.
// ------------------------------------------------------------------

void update_vertex(
    const WPA& wpa,
    DStarPlanner& planner,
    unsigned u,
    unsigned sstart
){
    double g_u   = get_g(planner.g,   u);
    double rhs_u = get_rhs(planner.rhs, u);

    // {07}/{08}: inconsistent — insert/update with new key
    if (g_u != rhs_u) {
        DStarKey key = calculate_key(planner, u, sstart);
        std::cout << "[DBG update_vertex] state=" << u
                  << " g=" << g_u << " rhs=" << rhs_u
                  << " key=(" << key.first << "," << key.second << ")\n";
        planner.U.push({key, u});
    }
    // {09}: consistent — lazy deletion handles removal on pop
}

// ------------------------------------------------------------------
// dstar_replan  (Figure 4, lines 36-48)
//
// {38} km = km + h(slast, sstart)
// {39} slast = sstart
// {40} for all directed edges (u,v) with changed edge costs
// {41}   cold = c(u,v)
// {42}   Update the edge cost c(u,v)
// {43}   if cold > c(u,v)                    <- cost decreased (unblocked)
// {44}     if u != sgoal: rhs(u) = min(rhs(u), c(u,v) + g(v))
// {45}   else if rhs(u) == cold + g(v)       <- cost increased (blocked)
// {46}     if u != sgoal: rhs(u) = min over Succ(u) of (c(u,s') + g(s'))
// {47}   UpdateVertex(u)
// {48} ComputeShortestPath()
// ------------------------------------------------------------------

LassoResult dstar_replan(
    WPA& wpa,
    DStarPlanner& planner,
    unsigned current,
    const std::vector<unsigned>& changed_states,
    bool is_now_blocked,
    ReplanMode mode
){
    if (mode == ReplanMode::FULL_RECOMPUTE) {
        planner = make_planner(wpa, mode);
        return dstar_plan(wpa, planner, current);
    }

    // {38} km += h(slast, sstart) — h=0 for now (no heuristic yet)
    planner.km += 0.0;
    // {39} slast = sstart
    planner.slast = current;

    double new_cost = is_now_blocked
        ? std::numeric_limits<double>::infinity()
        : 1.0;
    double cold = is_now_blocked ? 1.0 : std::numeric_limits<double>::infinity();

    std::ofstream& log = dbg("replan.log");
    log << "\n========================================================\n";
    log << "[replan] current=" << current
        << " pos=(" << wpa.pos_of(current).x << "," << wpa.pos_of(current).y << ")"
        << " nba=" << wpa.nba_state_of(current)
        << " is_now_blocked=" << is_now_blocked
        << " new_cost=" << new_cost << " cold=" << cold
        << " #changed=" << changed_states.size() << "\n";

    // {40-47} for each changed product state
    for (unsigned u : changed_states) {
        // {42} update edge cost in WPA
        wpa.set_state_exit_weight(u, new_cost);

        if (u == planner.s_imag) continue;

        double rhs_before = get_rhs(planner.rhs, u);
        double g_before   = get_g(planner.g, u);

        // {43-44} cost decreased — rhs(u) might improve
        if (cold > new_cost) {
            for (const WPA::Neighbor& nb : wpa.neighbors_ext(u)) {
                double candidate = nb.cost + get_g(planner.g, nb.dst);
                if (candidate < get_rhs(planner.rhs, u)) {
                    planner.rhs[u] = candidate;
                }
            }
        }
        // {45-46} cost increased — recompute rhs(u) from all successors
        else {
            double new_rhs = std::numeric_limits<double>::infinity();
            for (const WPA::Neighbor& nb : wpa.neighbors_ext(u)) {
                new_rhs = std::min(new_rhs, nb.cost + get_g(planner.g, nb.dst));
            }
            if (wpa.is_accepting(u) && planner.cycle_cost.count(u) > 0) {
                new_rhs = std::min(new_rhs, planner.cycle_cost[u] + get_g(planner.g, planner.s_imag));
            }
            planner.rhs[u] = new_rhs;
        }

        // log the edge update for this changed state
        log << "  [changed] u=" << u
            << " pos=(" << wpa.pos_of(u).x << "," << wpa.pos_of(u).y << ")"
            << " nba=" << wpa.nba_state_of(u)
            << " branch=" << (cold > new_cost ? "DECREASE" : "INCREASE")
            << " g=" << g_before
            << " rhs:" << rhs_before << "->" << get_rhs(planner.rhs, u) << "\n";
        for (const WPA::Neighbor& nb : wpa.neighbors_ext(u)) {
            log << "      succ " << nb.dst
                << " pos=(" << wpa.pos_of(nb.dst).x << "," << wpa.pos_of(nb.dst).y << ")"
                << " nba=" << wpa.nba_state_of(nb.dst)
                << " c=" << nb.cost
                << " g[succ]=" << get_g(planner.g, nb.dst) << "\n";
        }

        // {47} update_vertex
        update_vertex(wpa, planner, u, current);
    }

    // OPTION 1 suffix repair (stopgap). Must run AFTER the edge weights above
    // are applied (it re-searches on the updated graph) and BEFORE the prefix
    // pass below, so the coupling's rhs(acc) updates are folded into the same
    // compute_shortest_path. MIGRATION: this becomes Alg. 2 SUFFIXREPLAN per
    // accepting state — see recompute_affected_cycles() and MIGRATION_NOTES.md.
    recompute_affected_cycles(wpa, planner, changed_states, current);

    // {48} recompute shortest path (prefix). Picks up both the changed real
    // edges and the coupled virtual-edge (cycle-cost) updates in one pass.
    compute_shortest_path(wpa, planner, planner.pred_map, current, planner.s_imag);

    // dump g/rhs for every product state so we can see what survived the replan
    log << "[replan] g/rhs after compute_shortest_path (current=" << current << "):\n";
    for (unsigned s = 0; s < wpa.prod()->num_states(); ++s) {
        double gs  = get_g(planner.g, s);
        double rhs = get_rhs(planner.rhs, s);
        log << "  s=" << s
            << " pos=(" << wpa.pos_of(s).x << "," << wpa.pos_of(s).y << ")"
            << " nba=" << wpa.nba_state_of(s)
            << " g=" << gs << " rhs=" << rhs
            << (gs != rhs ? " INCONSISTENT" : "") << "\n";
    }
    log << std::flush;

    return dstar_plan(wpa, planner, current);
}
