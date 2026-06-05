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
// dstar_replan  (Phase 2c — TODO)
// ------------------------------------------------------------------

LassoResult dstar_replan(
    const WPA& wpa,
    DStarPlanner& planner,
    unsigned current,
    const std::vector<unsigned>& changed_states,
    ReplanMode mode
){
    // TODO: implement incremental replanning (Phase 2c)
    return {};
}
