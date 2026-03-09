/*
    For the start have it run A* and return the full path

    takes in product from the ts.cpp and then runs D*
    Should return next step probably
    - volitile memory to keep track of the closet paths?
    - something?
*/

#include <iostream>
#include <vector>
#include <algorithm>

#include "robot.hpp"
#include "grid_world.hpp"
#include "ts.hpp"
#include "dstar.hpp"


/*
-- from the paper
Alg 3 LTL-D*

Const G(ltl) and init()

construct s_img

for accepting_state for all F' do
    minimal_cost_loop, cost[accepting_state] = SUFFICINIT(G(ltl), k)
    insert <accpeting_state, s_img> with the weigth cost cost[accpting _state]

ComputerShortestPath(start_state, s_img)
Retrieve o_pre, o_suf

while(True)
    Move to state_next. s_start = state_next
    scan graph for a set mod containing changed edges
    if mod then
        for accpeting_state for all F' do
            o_pre, cost = SUFFIXREPLAIN(mod, k)
            if cost[s_k_acc] is updated then
                UPDATE_VERTEX(accpeting_state)
        if km in o_pre
            update
        else
            Initizalize()
        for <u,v> for all mod:
            update the weight with Cost(u,v)
            UpdateVertex(u)
        ComputeShortestPath(state_start, s_img)
        retrive o_pre and o_suf

*/



/*
ASTAR for main and only 1 path
 - static visulization
*/
std::vector<Pos> astar_find_path(
    spot::twa_graph_ptr product, GridWorld world
){
    // HOW TO RUN ALGORITHM ON GRAPH


    // HARDCODED EX
    std::vector<Pos> path = {
        {0,0}, {1,0}, {2,0}, {2,1}, {2,2}, {3,2}, {4,2}
    };
    return path;
}