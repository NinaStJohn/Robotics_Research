/*
    For the start have it run A* and return the full path

    takes in product from the ts.cpp and then runs D*
    Should return next step probably
    - volitile memory to keep track of the closet paths?
    - something?
*/



#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>

#include "dstar.hpp"

// ------------------------------------------------------------------
// Helpers
// ------------------------------------------------------------------

static double euclidean(Pos a, Pos b) {
    double dx = static_cast<double>(a.x - b.x);
    double dy = static_cast<double>(a.y - b.y);
    return std::sqrt(dx*dx + dy*dy);
}

// A* open set entry
struct Node {
    double   f;       // g + h
    double   g;       // cost from init
    unsigned state;

    // min-heap: smallest f first
    bool operator>(const Node& o) const { return f > o.f; }
};

// ------------------------------------------------------------------
// A*
//
// Finds shortest path in the product automaton from init_state to
// any accepting state.
//
// Returns the path as grid Pos sequence (via wpa.pos_of).
//
// TODO: this only finds the prefix of the lasso.  To recover the
//       full infinite accepting run (prefix + cycle) required by
//       LTL semantics, run a second A* from the accepting state back
//       to itself.  See Ren et al. (LTL-D*) Alg 3 SUFFIXINIT for
//       the cycle cost computation.
//
// TODO: for D* Lite / replanning (Ren et al.), replace this with an
//       incremental search that updates edge costs on environment
//       change rather than replanning from scratch.
// ------------------------------------------------------------------
std::vector<Pos> astar_find_path(const WPA& wpa, const GridWorld& world)
{
    unsigned init = wpa.init_state();

    // Early out: start is already accepting
    if (wpa.is_accepting(init)) {
        return { wpa.pos_of(init) };
    }

    // g-cost map
    std::unordered_map<unsigned, double> g_cost;
    g_cost[init] = 0.0;

    // parent map for path reconstruction
    std::unordered_map<unsigned, unsigned> parent;

    // open set
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

    // heuristic: Euclidean distance to nearest accepting state.
    // TODO: precompute the set of accepting state positions and find
    //       the true nearest one.  Right now we have no cheap way to
    //       enumerate accepting states without a full scan, so we use
    //       zero heuristic for the first pass (still correct, just
    //       degrades to Dijkstra).  Replace once accepting state list
    //       is exposed from WPA.
    auto h = [](Pos /*p*/) -> double { return 0.0; };

    open.push({ h(wpa.pos_of(init)), 0.0, init });

    while (!open.empty()) {
        Node cur = open.top(); open.pop();

        // stale entry check
        auto it = g_cost.find(cur.state);
        if (it != g_cost.end() && cur.g > it->second)
            continue;

        unsigned nba_component = wpa.nba_state_of(cur.state);
        Pos p = wpa.pos_of(cur.state);
        if (nba_component == 0 && world.has_label(p, "b")) {
            // Reconstruct path of product states -> Pos
            std::vector<Pos> path;
            unsigned s = cur.state;
            while (true) {
                path.push_back(wpa.pos_of(s));
                if (s == init) break;
                s = parent.at(s);
            }
            std::reverse(path.begin(), path.end());

            // Deduplicate consecutive identical positions
            // (caused by NBA transitions that don't move in the grid)
            std::vector<Pos> deduped;
            for (const auto& pos : path) {
                if (deduped.empty() ||
                    deduped.back().x != pos.x ||
                    deduped.back().y != pos.y)
                    deduped.push_back(pos);
            }
            return deduped;
        }

        for (const auto& [dst, cost] : wpa.neighbors(cur.state)) {
            double tentative_g = cur.g + cost;

            auto it = g_cost.find(dst);
            if (it != g_cost.end() && tentative_g >= it->second)
                continue;

            g_cost[dst]  = tentative_g;
            parent[dst]  = cur.state;

            double f = tentative_g + h(wpa.pos_of(dst));
            open.push({ f, tentative_g, dst });
        }
    }

    std::cerr << "astar_find_path(): no accepting state reachable\n";
    return {};
}