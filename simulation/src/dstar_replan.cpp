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
    [[maybe_unused]] const WPA& wpa,   // kept for API symmetry; needed once the
    DStarSearch& search,                // heuristic in calculate_key uses positions
    unsigned u,
    unsigned sstart
){
    double g_u   = get_g(search.g,   u);
    double rhs_u = get_rhs(search.rhs, u);

    // {07}/{08}: inconsistent — insert/update with new key
    if (g_u != rhs_u) {
        DStarKey key = calculate_key(search, u, sstart);
        std::cout << "[DBG update_vertex] state=" << u
                  << " g=" << g_u << " rhs=" << rhs_u
                  << " key=(" << key.first << "," << key.second << ")\n";
        search.U.push({key, u});
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
//
// Each StateChange carries its own cold/new_cost (see dstar.hpp) instead of
// one shared direction for the whole batch — required once changes can
// come from a radius-based sensor reveal, which can report several cells at
// once in mixed directions (some newly blocked, others newly cleared),
// unlike the old single mouse-click trigger which only ever changed one
// cell in one direction per call.
// ------------------------------------------------------------------

LassoResult dstar_replan(
    WPA& wpa,
    DStarPlanner& planner,
    unsigned current,
    const std::vector<StateChange>& changed_states,
    ReplanMode mode
){
    if (mode == ReplanMode::FULL_RECOMPUTE) {
        planner = make_planner(wpa, mode, planner.suffix_mode);
        return dstar_plan(wpa, planner, current);
    }

    // {38} km += h(slast, sstart) — h=0 for now (no heuristic yet)
    planner.prefix.km += 0.0;
    // {39} slast = sstart
    planner.slast = current;

    std::ofstream& log = dbg("replan.log");
    log << "\n========================================================\n";
    log << "[replan] current=" << current
        << " pos=(" << wpa.pos_of(current).x << "," << wpa.pos_of(current).y << ")"
        << " nba=" << wpa.nba_state_of(current)
        << " #changed=" << changed_states.size() << "\n";

    // {40-47} for each changed product state
    for (const StateChange& sc : changed_states) {
        unsigned u        = sc.state;
        double   cold     = sc.cold;
        double   new_cost = sc.new_cost;

        // {42} update edge cost in WPA
        wpa.set_state_exit_weight(u, new_cost);

        if (u == planner.prefix.goal) continue;

        double rhs_before = get_rhs(planner.prefix.rhs, u);
        double g_before   = get_g(planner.prefix.g, u);

        // {43-46} — shared with suffixreplan()'s per-accepting-state repair.
        repair_rhs_for_changed_edge(wpa, planner, planner.prefix, u, cold, new_cost);

        // log the edge update for this changed state
        log << "  [changed] u=" << u
            << " pos=(" << wpa.pos_of(u).x << "," << wpa.pos_of(u).y << ")"
            << " nba=" << wpa.nba_state_of(u)
            << " branch=" << (cold > new_cost ? "DECREASE" : "INCREASE")
            << " cold=" << cold << " new_cost=" << new_cost
            << " g=" << g_before
            << " rhs:" << rhs_before << "->" << get_rhs(planner.prefix.rhs, u) << "\n";
        for (const WPA::Neighbor& nb : wpa.neighbors_ext(u)) {
            log << "      succ " << nb.dst
                << " pos=(" << wpa.pos_of(nb.dst).x << "," << wpa.pos_of(nb.dst).y << ")"
                << " nba=" << wpa.nba_state_of(nb.dst)
                << " c=" << nb.cost
                << " g[succ]=" << get_g(planner.prefix.g, nb.dst) << "\n";
        }

        // {47} update_vertex
        update_vertex(wpa, planner.prefix, u, current);
    }

    // Suffix repair. Must run AFTER the edge weights above are applied (it
    // re-searches/repairs on the updated graph) and BEFORE the prefix pass
    // below, so the coupling's rhs(acc) updates are folded into the same
    // compute_shortest_path.
    //
    // suffix_mode lets Option 1 (stopgap, full A* re-search) and Option 2
    // (paper-faithful incremental SUFFIXREPLAN) be A/B'd on the same
    // scenario — see SuffixMode in dstar.hpp.
    switch (planner.suffix_mode) {
        case SuffixMode::OPTION2_INCREMENTAL:
            suffixreplan(wpa, planner, changed_states, current);
            break;
        case SuffixMode::OPTION1_ASTAR:
        default:
            recompute_affected_cycles(wpa, planner, changed_states, current);
            break;
    }

    // {48} recompute shortest path (prefix). Picks up both the changed real
    // edges and the coupled virtual-edge (cycle-cost) updates in one pass.
    compute_shortest_path(wpa, planner, planner.prefix, planner.prefix.pred_map, current, planner.prefix.goal);

    // dump g/rhs for every product state so we can see what survived the replan
    log << "[replan] g/rhs after compute_shortest_path (current=" << current << "):\n";
    for (unsigned s = 0; s < wpa.prod()->num_states(); ++s) {
        double gs  = get_g(planner.prefix.g, s);
        double rhs = get_rhs(planner.prefix.rhs, s);
        log << "  s=" << s
            << " pos=(" << wpa.pos_of(s).x << "," << wpa.pos_of(s).y << ")"
            << " nba=" << wpa.nba_state_of(s)
            << " g=" << gs << " rhs=" << rhs
            << (gs != rhs ? " INCONSISTENT" : "") << "\n";
    }
    log << std::flush;

    return dstar_plan(wpa, planner, current);
}
