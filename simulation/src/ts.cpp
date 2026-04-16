/*

    This is a HEAVY LIFTER FILE

    This file is reposnsible for taking in the built world and
    translating it to LTLs

    Big world to abstraction file
    takes in world and makes a TS
    takes in bot and makes a Automa
    takes product and returns product

    TS_update function
    - should take in new world
    - find point of difference
    - update the edges of the effected area
    - ex.  auto -> new_edge(0,1,p1)


*/
#include "ts.hpp"
#include "grid_world.hpp"
#include "robot.hpp"

#include <iostream>

// LTL parsing
#include <spot/tl/parse.hh>
#include <spot/tl/formula.hh>

// viz
#include <spot/tl/print.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twaalgos/dot.hh>
#include <fstream>
#include <string>
#include <filesystem>
#include <cstdlib>

// NBA translation + product
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/product.hh>

// making TS
#include <spot/twa/twagraph.hh>
#include <spot/twa/bdddict.hh>
#include <bddx.h>


// -----------------------------------------------------
// Forward declarations (private)
// -----------------------------------------------------
 
// ts_state_to_pos is populated here so A* can recover Pos from state id
static spot::twa_graph_ptr world_to_ts(
    const GridWorld&                       world,
    const Robot&                           robot,
    const spot::bdd_dict_ptr&              dict,
    std::unordered_map<unsigned, Pos>&     out_ts_state_to_pos,
    std::unordered_map<Robot::Action, int>& out_act_ap,
    std::unordered_map<std::string, int>&  out_world_ap
);
 
static spot::twa_graph_ptr ltl_to_nba(
    const spot::formula&      f,
    const spot::bdd_dict_ptr& dict
);
 
static bdd label_of_cell(
    const GridWorld&                            world,
    const Pos&                                  p,
    const std::unordered_map<std::string, int>& ap_index
);
 
static bdd action_label_bdd(
    Robot::Action                                    a,
    const std::unordered_map<Robot::Action, int>&    apmap
);
 
static std::unordered_map<std::string, int>
register_world_aps(const GridWorld& world, const spot::twa_graph_ptr& ts);
 
static std::unordered_map<Robot::Action, int>
register_action_aps(spot::twa_graph_ptr ts, const Robot& robot);
 
static std::string action_ap_name(Robot::Action a);
 
static void write_dot_and_pdf(
    const spot::const_twa_ptr& aut,
    const std::string&         base_path
);
 
 
// -----------------------------------------------------
// Public entry point
// -----------------------------------------------------
 
ProductBundle build_product_from_world_robot_ltl(
    const GridWorld&   world,
    const Robot&       robot,
    const std::string& ltl_infix
){
    spot::bdd_dict_ptr dict = spot::make_bdd_dict();
 
    // 1) Parse LTL
    spot::parsed_formula pf = spot::parse_infix_psl(ltl_infix);
    if (pf.format_errors(std::cerr))
        return ProductBundle{};   // null bundle on parse error
 
    // 2) Build TS — populate bundle maps in place
    ProductBundle bundle;
    spot::twa_graph_ptr ts = world_to_ts(
        world, robot, dict,
        bundle.ts_state_to_pos,
        bundle.act_ap,
        bundle.world_ap
    );
 
    // 3) Build NBA
    spot::twa_graph_ptr nba = ltl_to_nba(pf.f, dict);
    bundle.nba_size = static_cast<unsigned>(nba->num_states());

    // 4) Product automaton  (TS x NBA)
    bundle.prod = spot::product(ts, nba);

    // 5) Remap ts_state_to_pos from TS-state indices to product-state indices.
    //    Spot BFS-numbers product states by reachability — the formula
    //    (ts_state * nba_size + nba_state) is NOT valid after compaction.
    //    The "product-states" named property stores the original (left, right)
    //    pair for every product state.
    {
        using pairs_t = std::vector<std::pair<unsigned, unsigned>>;
        auto* ps = bundle.prod->get_named_prop<pairs_t>("product-states");
        if (ps && !ps->empty()) {
            std::unordered_map<unsigned, Pos>      prod_to_pos;
            std::unordered_map<unsigned, unsigned> prod_to_nba;
            for (unsigned s = 0; s < bundle.prod->num_states(); ++s) {
                unsigned ts_s  = (*ps)[s].first;
                unsigned nba_s = (*ps)[s].second;
                prod_to_nba[s] = nba_s;
                auto it = bundle.ts_state_to_pos.find(ts_s);
                if (it != bundle.ts_state_to_pos.end())
                    prod_to_pos[s] = it->second;
            }
            bundle.ts_state_to_pos  = std::move(prod_to_pos);
            bundle.prod_state_to_nba = std::move(prod_to_nba);
        } else {
            std::cerr << "ts.cpp: \"product-states\" property missing — "
                         "pos_of() will return garbage\n";
        }
    }
 
    write_dot_and_pdf(ts,          "output/ts");
    write_dot_and_pdf(nba,         "output/nba");
    write_dot_and_pdf(bundle.prod, "output/prod");
 
    return bundle;
}
 
 
// -----------------------------------------------------
// NBA
// -----------------------------------------------------
 
static spot::twa_graph_ptr ltl_to_nba(
    const spot::formula&      f,
    const spot::bdd_dict_ptr& dict
){
    spot::translator trans(dict);
    trans.set_type(spot::postprocessor::BA);
    trans.set_pref(spot::postprocessor::Small);
    return trans.run(f);
}
 
 
// -----------------------------------------------------
// TS construction
// -----------------------------------------------------
 
static std::string apset_string_for_cell(const GridWorld& world, int x, int y)
{
    Pos p{x, y};
    std::vector<std::string> names = world.label_names();
    std::sort(names.begin(), names.end());
 
    std::string s;
    for (const auto& nm : names) {
        if (world.has_label(p, nm)) {
            if (!s.empty()) s += ",";
            s += nm;
        }
    }
    return s.empty() ? "∅" : s;
}
 
static spot::twa_graph_ptr world_to_ts(
    const GridWorld&                        world,
    const Robot&                            robot,
    const spot::bdd_dict_ptr&               dict,
    std::unordered_map<unsigned, Pos>&      out_ts_state_to_pos,
    std::unordered_map<Robot::Action, int>& out_act_ap,
    std::unordered_map<std::string, int>&   out_world_ap
){
    spot::twa_graph_ptr ts = spot::make_twa_graph(dict);
 
    auto* state_names =
        ts->get_or_set_named_prop<std::vector<std::string>>("state-names");
 
    // 1) Register APs
    out_act_ap   = register_action_aps(ts, robot);
    out_world_ap = register_world_aps(world, ts);
 
    // 2) Allocate states, build (x,y) -> sid and sid -> Pos maps
    int w = world.width();
    int h = world.height();
 
    std::vector<int> id(static_cast<std::size_t>(w * h), -1);
 
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (world.is_blocked({x, y})) continue;
 
            unsigned sid = ts->new_state();
            id[static_cast<std::size_t>(y * w + x)] = static_cast<int>(sid);
 
            // reverse map: TS state -> grid pos  (used by A* heuristic)
            out_ts_state_to_pos[sid] = Pos{x, y};
 
            if (state_names->size() <= sid)
                state_names->resize(sid + 1);
            (*state_names)[sid] = apset_string_for_cell(world, x, y);
        }
    }
 
    // 3) Initial state
    Pos start = robot.position();
    if (!world.in_bounds(start) || world.is_blocked(start)) {
        std::cerr << "world_to_ts(): robot start is out of bounds or blocked\n";
        return spot::twa_graph_ptr();
    }
 
    int init_id = id[static_cast<std::size_t>(start.y * w + start.x)];
    if (init_id < 0) {
        std::cerr << "world_to_ts(): no TS state for start cell\n";
        return spot::twa_graph_ptr();
    }
    ts->set_init_state(static_cast<unsigned>(init_id));
 
    // 4) Edges
    const auto& acts = robot.actions();
 
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int s = id[static_cast<std::size_t>(y * w + x)];
            if (s < 0) continue;
 
            for (Robot::Action a : acts) {
                int nx = x, ny = y;
 
                if      (a == Robot::Action::Left)  nx = x - 1;
                else if (a == Robot::Action::Right) nx = x + 1;
                else if (a == Robot::Action::Up)    ny = y - 1;
                else if (a == Robot::Action::Down)  ny = y + 1;
                // Stay: nx,ny unchanged
 
                if (!world.in_bounds({nx, ny})) continue;
                if (world.is_blocked({nx, ny}))  continue;
 
                int t = id[static_cast<std::size_t>(ny * w + nx)];
                if (t < 0) continue;
 
                bdd actbdd = action_label_bdd(a, out_act_ap);
                bdd stbdd  = label_of_cell(world, {nx, ny}, out_world_ap);
                ts->new_edge(static_cast<unsigned>(s),
                             static_cast<unsigned>(t),
                             bdd_and(actbdd, stbdd));
            }
        }
    }
 
    return ts;
}
 
 
// -----------------------------------------------------
// BDD helpers
// -----------------------------------------------------
 
static bdd label_of_cell(
    const GridWorld&                            world,
    const Pos&                                  p,
    const std::unordered_map<std::string, int>& ap_index
){
    bdd out = bddtrue;
    for (const auto& kv : ap_index) {
        bool val = world.has_label(p, kv.first);
        out &= val ? bdd_ithvar(kv.second) : bdd_nithvar(kv.second);
    }
    return out;
}
 
static bdd action_label_bdd(
    Robot::Action                                 a,
    const std::unordered_map<Robot::Action, int>& apmap
){
    auto it = apmap.find(a);
    if (it == apmap.end()) return bddfalse;
    return bdd_ithvar(it->second);
}
 
 
// -----------------------------------------------------
// AP registration
// -----------------------------------------------------
 
static std::unordered_map<std::string, int>
register_world_aps(const GridWorld& world, const spot::twa_graph_ptr& ts)
{
    std::unordered_map<std::string, int> ap_index;
    for (const auto& kv : world.label_map())
        ap_index[kv.first] = ts->register_ap(kv.first);
    return ap_index;
}
 
static std::string action_ap_name(Robot::Action a)
{
    if (a == Robot::Action::Stay)  return "S";
    if (a == Robot::Action::Left)  return "L";
    if (a == Robot::Action::Right) return "R";
    if (a == Robot::Action::Up)    return "U";
    if (a == Robot::Action::Down)  return "D";
    return "UNK";
}
 
static std::unordered_map<Robot::Action, int>
register_action_aps(spot::twa_graph_ptr ts, const Robot& robot)
{
    std::unordered_map<Robot::Action, int> m;
    for (Robot::Action a : robot.actions())
        m[a] = ts->register_ap(action_ap_name(a));
    return m;
}
 
 
// -----------------------------------------------------
// Viz
// -----------------------------------------------------
 
static void write_dot_and_pdf(
    const spot::const_twa_ptr& aut,
    const std::string&         base_path
){
    std::string dot_file = base_path + ".dot";
    std::string pdf_file = base_path + ".pdf";
 
    std::ofstream os(dot_file);
    if (!os) { std::cerr << "Could not open " << dot_file << "\n"; return; }
 
    spot::print_dot(os, aut);
    os.close();
 
    int rc = std::system(("dot -Tpdf " + dot_file + " -o " + pdf_file).c_str());
    if (rc != 0) std::cerr << "dot command failed (rc=" << rc << ")\n";
}