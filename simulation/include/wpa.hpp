#pragma once

#include <vector>
#include <unordered_map>
#include <utility>

#include "ts.hpp"
#include "types.hpp"

// ------------------------------------------------------------------
// WPA  (Weighted Product Automaton)
//
// Wraps ProductBundle and attaches a cost to every edge in the
// product graph.  A* (dstar.cpp) operates entirely through this
// interface.
//
// TODO: when scaling to multi-robot, replace owned ProductBundle
//       with a shared_ptr<const ProductBundle> so multiple robots
//       can reference the same product without copying.
// ------------------------------------------------------------------
class WPA {
public:
    // Takes ownership of the bundle.
    // Initializes all edge weights to 1.0.
    explicit WPA(ProductBundle&& bundle);

    // -- Graph interface for A* --

    unsigned init_state() const;

    // Returns (dst_state, edge_cost) for every edge out of state_id.
    std::vector<std::pair<unsigned, double>>
    neighbors(unsigned state_id) const;

    // updated neighbor function that also return via_accepting bool
    struct Neighbor { unsigned dst; double cost; bool accepting; };
    std::vector<Neighbor> neighbors_ext(unsigned state_id) const;

    // True if state_id is an accepting state in the product automaton.
    bool is_accepting(unsigned state_id) const;

    // Recovers grid Pos from a product state id.
    // Direct lookup into product-state-indexed ts_state_to_pos.
    Pos pos_of(unsigned state_id) const;

    // Returns the NBA component of a product state.
    unsigned nba_state_of(unsigned state_id) const;

    // -- Weight control --

    // Override the cost of a specific edge (by spot edge index).
    // Use for heterogeneous robot costs, restricted zones, etc.
    void set_weight(unsigned edge_idx, double cost);

    // Convenience: set cost for every edge leaving a given state.
    void set_state_exit_weight(unsigned state_id, double cost);

    unsigned nba_size() const { return nba_size_; }

    // DEBUG: remove before scaling
    const spot::twa_graph_ptr& prod() const { return bundle_.prod; }
private:
    ProductBundle bundle_;
    unsigned      nba_size_;   // cached at construction for pos_of()

    // edge index -> cost
    // TODO: consider flat vector<double> indexed by edge_idx for
    //       cache performance once edge counts get large.
    std::unordered_map<unsigned, double> weights_;
};