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

// NBA translation + product
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/product.hh>



spot::twa_graph_ptr build_product_from_world_robot_ltl(const GridWorld& world,
                                                       const Robot& robot,
                                                       const std::string& ltl_infix)
{
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
    spot::twa_graph_ptr prod = spot::twa_product(ts, nba);

    return prod;
}

static spot::twa_graph_ptr ltl_to_nba(const spot::formula& f,
                                     const spot::bdd_dict_ptr& dict)
{
    spot::translator trans(dict);
    trans.set_type(spot::postprocessor::BA);
    trans.set_pref(spot::postprocessor::Small);
    spot::twa_graph_ptr nba = trans.run(f);
    return nba;
}i 

static spot::twa_graph_ptr world_to_ts(const GridWorld& world,
                                      const Robot& robot,
                                      const spot::bdd_dict_ptr& dict);
