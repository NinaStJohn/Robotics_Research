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

    // {40-47} for each changed product state
    for (unsigned u : changed_states) {
        // {42} update edge cost in WPA
        wpa.set_state_exit_weight(u, new_cost);

        if (u == planner.s_imag) continue;

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

        // {47} update_vertex
        update_vertex(wpa, planner, u, current);
    }

    // {48} recompute shortest path
    compute_shortest_path(wpa, planner, planner.pred_map, current, planner.s_imag);

    return dstar_plan(wpa, planner, current);
}
