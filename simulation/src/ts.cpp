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

#include <algorithm>

// Private functions
static spot::twa_graph_ptr world_to_ts(
    const GridWorld& world,
    const Robot& robot,
    const spot::bdd_dict_ptr& dict
);

static spot::twa_graph_ptr ltl_to_nba(
    const spot::formula& f,
    const spot::bdd_dict_ptr& dict
);

static bdd label_of_cell(
    const GridWorld& world,
    const GridWorld::Pos& p,
    const std::unordered_map<std::string,int>& ap_index
);

static bdd action_label_bdd(
    Robot::Action a,
    const std::unordered_map<Robot::Action, int>& apmap
);

// labeling registering
static std::unordered_map<std::string,int>
register_world_aps(const GridWorld& world,
                   const spot::twa_graph_ptr& ts);

static std::string action_ap_name(Robot::Action a);

// vis
static void write_dot_and_pdf(
    const spot::const_twa_ptr& aut,
    const std::string& base_path
);

static std::unordered_map<Robot::Action, int>
register_action_aps(spot::twa_graph_ptr ts, const Robot& robot);




/* 
 * Functions 
 */

// main entry point for main
spot::twa_graph_ptr build_product_from_world_robot_ltl(
    const GridWorld& world,
    const Robot& robot,
    const std::string& ltl_infix
){
    // 0) One dict for EVERYTHING in this pipeline
    spot::bdd_dict_ptr dict = spot::make_bdd_dict();

    // 1) Parse LTL here (so main stays dumb)
    spot::parsed_formula pf = spot::parse_infix_psl(ltl_infix);
    if (pf.format_errors(std::cerr)) {
        return spot::twa_graph_ptr(); // null
    }
    spot::formula f = pf.f;

    // 2) Build TS (this is where robot actions interact with world)
    spot::twa_graph_ptr ts = world_to_ts(world, robot, dict);

    // 3) Build NBA from LTL (must share dict)
    spot::twa_graph_ptr nba = ltl_to_nba(f, dict);

    // 4) Product
    // https://spot.lre.epita.fr/doxygen/classspot_1_1twa__product.html
    // spot::twa_graph_ptr prod = spot::twa_product(ts, nba);
    spot::twa_graph_ptr prod = spot::product(ts, nba);

    // just have everything be visulized
    write_dot_and_pdf(ts,   "output/ts");
    write_dot_and_pdf(nba,  "output/nba");
    write_dot_and_pdf(prod, "output/prod");

    return prod;
}



static spot::twa_graph_ptr ltl_to_nba(
    const spot::formula& f,
    const spot::bdd_dict_ptr& dict
){
    spot::translator trans(dict);
    trans.set_type(spot::postprocessor::BA);
    trans.set_pref(spot::postprocessor::Small);
    spot::twa_graph_ptr nba = trans.run(f);
    return nba;
}

// -----------------------------------------------------
// Making the world and robot in TS
//
//
// -----------------------------------------------------

static std::string apset_string_for_cell(
    const GridWorld& world, int x, int y
){
    GridWorld::Pos p = {x, y};
    std::vector<std::string> names = world.label_names();

    // Optional but recommended: deterministic order for hand-checking.
    std::sort(names.begin(), names.end());
    std::string s;
    std::size_t i;
    for (i = 0; i < names.size(); ++i)
    {
        const std::string& nm = names[i];
        if (world.has_label(p, nm))
        {
            if (!s.empty()) s += ",";
            s += nm;
        }
    }
    if (s.empty()) s = "∅";
    return s;
}

static spot::twa_graph_ptr world_to_ts(
    const GridWorld& world,
    const Robot& robot,
    const spot::bdd_dict_ptr& dict
){
    spot::twa_graph_ptr ts = spot::make_twa_graph(dict);

    std::vector<std::string>* state_names =
        ts->get_or_set_named_prop<std::vector<std::string> >("state-names");

    // 1) Register APs used in TS labels
    std::unordered_map<Robot::Action, int> act_ap = register_action_aps(ts, robot);

    std::unordered_map<std::string,int> ap_index =
    register_world_aps(world, ts);

    // 2) Create mapping from (x,y) -> Spot state id
    int w = world.width();
    int h = world.height();

    std::vector<int> id;
    id.resize(static_cast<std::size_t>(w * h), -1);

    int x, y;
    for (y = 0; y < h; ++y) {
        for (x = 0; x < w; ++x) {
            if (!world.is_blocked(x, y)) {
                // label the state names
                int sid = ts->new_state();

                // IMPORTANT: store mapping so edges and init state work
                id[static_cast<std::size_t>(y * w + x)] = sid;

                // resize state_names if needed
                if (state_names->size() <= static_cast<std::size_t>(sid))
                    state_names->resize(static_cast<std::size_t>(sid) + 1);

                // name = label set only (no pos)
                std::string nm = apset_string_for_cell(world, x, y);
                (*state_names)[static_cast<std::size_t>(sid)] = nm;
            }
        }
    }

    // 3) Init state from robot start position
    Robot::Pos start = robot.position();
    int sx = start.first;
    int sy = start.second;

    if (!world.in_bounds(sx, sy) || world.is_blocked(sx, sy)) {
        std::cerr << "world_to_ts(): robot start is out of bounds or blocked\n";
        return spot::twa_graph_ptr(); // null
    }

    int init_id = id[static_cast<std::size_t>(sy * w + sx)];
    if (init_id < 0) {
        std::cerr << "world_to_ts(): no TS state allocated for start cell\n";
        return spot::twa_graph_ptr(); // null
    }

    ts->set_init_state(init_id);

    // 4) Add transitions for each cell and each robot action
    const std::vector<Robot::Action>& acts = robot.actions();

    for (y = 0; y < h; ++y) {
        for (x = 0; x < w; ++x) {

            int s = id[static_cast<std::size_t>(y * w + x)];
            if (s < 0) continue; // blocked/unallocated

            std::size_t i;
            for (i = 0; i < acts.size(); ++i) {

                Robot::Action a = acts[i];

                int nx = x;
                int ny = y;

                if (a == Robot::Action::Left)  nx = x - 1;
                else if (a == Robot::Action::Right) nx = x + 1;
                else if (a == Robot::Action::Up)    ny = y - 1;
                else if (a == Robot::Action::Down)  ny = y + 1;
                else if (a == Robot::Action::Stay)  { /* no change */ }

                if (!world.in_bounds(nx, ny)) continue;
                if (world.is_blocked(nx, ny)) continue;

                int t = id[static_cast<std::size_t>(ny * w + nx)];
                if (t < 0) continue;

                // Label edges with destination state's label (for product)
                bdd actbdd = action_label_bdd(a, act_ap);
                // destination cell
                GridWorld::Pos dstpos = {nx, ny};
                // world valuation at destination             
                bdd stbdd = label_of_cell(world, dstpos, ap_index); 

                bdd cond = bdd_and(actbdd, stbdd);
                ts->new_edge(s, t, cond);

            }
        }
    }

    return ts;
}



// label
static bdd label_of_cell(
    const GridWorld& world,
    const GridWorld::Pos& p,
    const std::unordered_map<std::string,int>& ap_index
){
    bdd out = bddtrue;

    std::unordered_map<std::string,int>::const_iterator it;

    for (it = ap_index.begin();
        it != ap_index.end();
        ++it)
    {
        const std::string& name = it->first;
        int ap = it->second;

        bool val = world.has_label(p, name);

        out &= (val ? bdd_ithvar(ap)
                    : bdd_nithvar(ap));
    }

    return out;
}

// THIS IS FOR VISULIZATION PURPOSES
static bdd action_label_bdd(
    Robot::Action a,
    const std::unordered_map<Robot::Action, int>& apmap)
{
    std::unordered_map<Robot::Action, int>::const_iterator it = apmap.find(a);
    if (it == apmap.end())
    {
        // action not supported -> label false (no edge should match this action)
        return bddfalse;
    }
    int apid = it->second;
    return bdd_ithvar(apid);
}


// -----------------------------------------------------
// Dynamic labeling of actions and states
// + registering
//
// -----------------------------------------------------

// state for grid world
static std::unordered_map<std::string,int>
register_world_aps(const GridWorld& world,
                   const spot::twa_graph_ptr& ts)
{
    std::unordered_map<std::string,int> ap_index;

    const std::unordered_map<std::string,int>& world_labels =
        world.label_map();

    std::unordered_map<std::string,int>::const_iterator it;

    for (it = world_labels.begin();
         it != world_labels.end();
         ++it)
    {
        const std::string& name = it->first;
        int ap = ts->register_ap(name);
        ap_index[name] = ap;
    }

    return ap_index;
}

// actions from robot actions
static std::string action_ap_name(
    Robot::Action a
){
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

    std::vector<Robot::Action> actions = robot.actions();
    for (std::size_t i = 0; i < actions.size(); ++i)
    {
        Robot::Action a = actions[i];
        std::string nm = action_ap_name(a); 
        int apid = ts->register_ap(nm);
        m.insert(std::make_pair(a, apid));
    }
    return m;
}



// -----------------------------------------------------
// Viz
//
//
// -----------------------------------------------------
static void write_dot_and_pdf(
    const spot::const_twa_ptr& aut,
    const std::string& base_path
)
{
    std::string dot_file = base_path + ".dot";
    std::string pdf_file = base_path + ".pdf";

    std::ofstream os(dot_file);
    if (!os) {
        std::cerr << "Could not open " << dot_file << "\n";
        return;
    }

    spot::print_dot(os, aut);
    os.close();

    std::string cmd = "dot -Tpdf " + dot_file + " -o " + pdf_file;
    int rc = std::system(cmd.c_str());
    if (rc != 0) {
        std::cerr << "dot command failed (rc=" << rc << ")\n";
    }
}

