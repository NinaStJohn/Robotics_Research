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



// Private functions
static spot::twa_graph_ptr world_to_ts(
    const GridWorld& world, const Robot& robot, const spot::bdd_dict_ptr& dict);
static spot::twa_graph_ptr ltl_to_nba(
    const spot::formula& f, const spot::bdd_dict_ptr& dict);
static bdd label_of_cell(int x, int y, int ap_a, int ap_b);
static bdd action_label_bdd(
    Robot::Action a,
    int ap_stay, 
    int ap_left, 
    int ap_right, 
    int ap_up, 
    int ap_down);
// vis helper
static void write_dot_and_pdf(
    const spot::const_twa_ptr& aut,
    const std::string& base_path
);


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


static spot::twa_graph_ptr world_to_ts(
    const GridWorld& world,
    const Robot& robot,
    const spot::bdd_dict_ptr& dict
){
    spot::twa_graph_ptr ts = spot::make_twa_graph(dict);

    // 1) Register APs used in TS labels
    int ap_a         = ts->register_ap("a");
    int ap_b         = ts->register_ap("b");
    int ap_act_stay  = ts->register_ap("act_stay");
    int ap_act_left  = ts->register_ap("act_left");
    int ap_act_right = ts->register_ap("act_right");
    int ap_act_up    = ts->register_ap("act_up");
    int ap_act_down  = ts->register_ap("act_down");
    // make this into a string

    // 2) Create mapping from (x,y) -> Spot state id
    int w = world.width();
    int h = world.height();

    std::vector<int> id;
    id.resize(static_cast<std::size_t>(w * h), -1);

    int x, y;
    for (y = 0; y < h; ++y) {
        for (x = 0; x < w; ++x) {
            if (!world.is_blocked(x, y)) {
                int sid = ts->new_state();
                id[static_cast<std::size_t>(y * w + x)] = sid;
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

                // Label edges with destination state's label
                bdd actbdd = action_label_bdd(a, ap_act_stay, ap_act_left, ap_act_right, ap_act_up, ap_act_down);
                bdd stbdd  = label_of_cell(nx, ny, ap_a, ap_b);   // or label_of_cell(x,y,...) depending on your convention
                bdd cond = actbdd & stbdd;
                ts->new_edge(s, t, cond);

                // debug print
                if (actbdd == bddfalse) {
                    std::cerr << "actbdd is FALSE for action\n";
                }
                if (stbdd == bddfalse) {
                    std::cerr << "stbdd is FALSE at (" << nx << "," << ny << ")\n";
                }
                if (cond == bddfalse) {
                    std::cerr << "cond is FALSE at (" << x << "," << y << ") -> ("
                            << nx << "," << ny << ")\n";
                }
            }
        }
    }

    return ts;
}

// label
static bdd label_of_cell(int x, int y, int ap_a, int ap_b)
{
    bool a_true = (x == 0 && y == 0);
    bool b_true = (x == 1 && y == 1);

    bdd out = bddtrue;
    out &= (a_true ? bdd_ithvar(ap_a) : bdd_nithvar(ap_a));
    out &= (b_true ? bdd_ithvar(ap_b) : bdd_nithvar(ap_b));
    return out;
}

static bdd action_label_bdd(
    Robot::Action a,
    int ap_stay, 
    int ap_left, 
    int ap_right, 
    int ap_up, 
    int ap_down
){
    bdd out = bddtrue;

    out &= (a == Robot::Action::Stay)  ? bdd_ithvar(ap_stay)  : bdd_nithvar(ap_stay);
    out &= (a == Robot::Action::Left)  ? bdd_ithvar(ap_left)  : bdd_nithvar(ap_left);
    out &= (a == Robot::Action::Right) ? bdd_ithvar(ap_right) : bdd_nithvar(ap_right);
    out &= (a == Robot::Action::Up)    ? bdd_ithvar(ap_up)    : bdd_nithvar(ap_up);
    out &= (a == Robot::Action::Down)  ? bdd_ithvar(ap_down)  : bdd_nithvar(ap_down);

    return out;
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
