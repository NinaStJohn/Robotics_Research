#include "wpa.hpp"

#include <climits>
#include <iostream>
#include <spot/twa/twagraph.hh>

// ------------------------------------------------------------------
// Constructor
// ------------------------------------------------------------------

WPA::WPA(ProductBundle&& bundle)
    : bundle_(std::move(bundle))
{
    const auto& prod = bundle_.prod;

    // Cache NBA size for pos_of() decomposition.
    // spot::product() encodes paired state as:
    //   product_state = ts_state * nba_size + nba_state
    // We recover nba_size by noting the product has ts_n * nba_n states.
    // We store it directly from the second automaton's state count.
    // spot does not expose the NBA handle post-product, so we derive it:
    //   nba_size = prod->num_states() / ts_num_states
    // where ts_num_states = ts_state_to_pos.size()
    //
    // NOTE: this assumes no unreachable states were pruned by spot.
    // If spot prunes states this will be wrong -- revisit if pos_of()
    // returns garbage.
    nba_size_ = bundle_.nba_size;

    // Initialize all edge weights to 1.0
    for (unsigned s = 0; s < prod->num_states(); ++s) {
        for (const auto& e : prod->out(s)) {
            unsigned idx = prod->edge_number(e);
            weights_[idx] = 1.0;
        }
    }
}


// ------------------------------------------------------------------
// Graph interface
// ------------------------------------------------------------------

unsigned WPA::init_state() const {
    return bundle_.prod->get_init_state_number();
}

std::vector<WPA::Neighbor> 
WPA::neighbors_ext(unsigned state_id) const
{
    std::vector<WPA::Neighbor> out;

    for (const spot::twa_graph::edge_storage_t& e : bundle_.prod->out(state_id)) {
        unsigned idx = bundle_.prod->edge_number(e);
        std::cout << "  edge to " << e.dst << " acc=" << e.acc << "\n";
        out.push_back({ e.dst, weights_.at(idx), is_accepting(e.dst) });
    }

    std::cout << "prod acc condition: num_sets=" 
          << bundle_.prod->acc().num_sets() << "\n";

    return out;
}

std::vector<std::pair<unsigned, double>>
WPA::neighbors(unsigned state_id) const
{
    std::vector<std::pair<unsigned, double>> out;

    for (const auto& e : bundle_.prod->out(state_id)) {
        unsigned idx  = bundle_.prod->edge_number(e);
        double   cost = weights_.at(idx);
        out.emplace_back(e.dst, cost);
    }

    return out;
}

bool WPA::is_accepting(unsigned state_id) const {
    // Spot uses transition-based acceptance (TBA).
    // An accepting edge s->t with acc.has(0) means the run is accepting
    // when that transition is *taken*, i.e. t is the goal, not s.
    // So we check if any *incoming* edge to state_id carries the mark
    // by checking all edges of all predecessors.
    // TODO: precompute predecessor map for performance at scale.
    for (const spot::twa_graph::edge_storage_t& e : bundle_.prod->out(state_id)) {
        if (e.acc.has(0)) return true;
    }
    return false;
}

Pos WPA::pos_of(unsigned state_id) const {
    auto it = bundle_.ts_state_to_pos.find(state_id);
    if (it == bundle_.ts_state_to_pos.end()) {
        std::cerr << "WPA::pos_of(): product state " << state_id
                  << " not in ts_state_to_pos\n";
        return Pos{-1, -1};
    }
    return it->second;
}

unsigned WPA::nba_state_of(unsigned state_id) const {
    auto it = bundle_.prod_state_to_nba.find(state_id);
    if (it == bundle_.prod_state_to_nba.end()) {
        std::cerr << "WPA::nba_state_of(): product state " << state_id << " not found\n";
        return UINT_MAX;
    }
    return it->second;
}


// ------------------------------------------------------------------
// Weight control
// ------------------------------------------------------------------

void WPA::set_weight(unsigned edge_idx, double cost) {
    weights_[edge_idx] = cost;
}

void WPA::set_state_exit_weight(unsigned state_id, double cost) {
    for (const auto& e : bundle_.prod->out(state_id)) {
        unsigned idx = bundle_.prod->edge_number(e);
        weights_[idx] = cost;
    }
}