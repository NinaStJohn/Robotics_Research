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
// astar_impl
//
// Prefix mode (is_cycle=false):
//   Runs A* from `start` to q_imag. Every accepting state gets a free
//   (cost 0) edge to q_imag, discovered on the fly.
//
// Cycle mode (is_cycle=true):
//   Runs A* from `start` back to `start`. Edges whose destination IS
//   `start` are intercepted and redirected to q_imag so that the same
//   reconstruction logic applies and parent[start] is never overwritten.
//
// Returns the path as product state IDs with q_imag excluded.
// ------------------------------------------------------------------
static std::vector<unsigned> astar_impl(const WPA& wpa, unsigned start, bool is_cycle = false)
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
            return path;
        }

        // stale entry check
        auto it = g_cost.find(cur.state);
        if (it != g_cost.end() && cur.g > it->second)
            continue;

        // real neighbors
        for (const WPA::Neighbor& nb : wpa.neighbors_ext(cur.state)) {
            // cycle mode: intercept edges back to start — redirect to q_imag
            // so parent[start] is never overwritten and reconstruction stays clean
            if (is_cycle && nb.dst == start) {
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

        // prefix only: free edge to q_imag for accepting states
        if (!is_cycle && cur.state != start && wpa.is_accepting(cur.state)) {
            std::cout << "[DBG] accepting: state=" << cur.state
                      << " nba=" << wpa.nba_state_of(cur.state)
                      << " pos=(" << wpa.pos_of(cur.state).x << ","
                      << wpa.pos_of(cur.state).y << ")"
                      << " g=" << cur.g << "\n";
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
// astar_find_path
//
// Entry point. Runs astar_impl twice:
//   1. prefix: init_state -> first accepting state
//   2. cycle:  that accepting state -> any accepting state (via q_imag)
// ------------------------------------------------------------------
LassoResult astar_find_path(const WPA& wpa)
{
    unsigned init = wpa.init_state();

    std::vector<unsigned> prefix_ids = astar_impl(wpa, init);
    if (prefix_ids.empty()) {
        std::cerr << "astar_find_path(): no accepting state reachable from init\n";
        return {};
    }

    std::cout << "[DBG] prefix state IDs: ";
    for (unsigned id : prefix_ids) std::cout << id << " ";
    std::cout << "\n";

    unsigned q_acc = prefix_ids.back();
    std::cout << "[DBG] cycle search starts from state=" << q_acc
              << " nba=" << wpa.nba_state_of(q_acc)
              << " pos=(" << wpa.pos_of(q_acc).x << "," << wpa.pos_of(q_acc).y << ")\n";

    std::vector<unsigned> cycle_ids = astar_impl(wpa, q_acc, true);
    if (cycle_ids.empty()) {
        std::cerr << "astar_find_path(): no cycle found from accepting state\n";
        return {};
    }

    std::cout << "[DBG] cycle state IDs: ";
    for (unsigned id : cycle_ids) std::cout << id << " ";
    std::cout << "\n";

    return { to_pos(wpa, prefix_ids), to_pos(wpa, cycle_ids) };
}
