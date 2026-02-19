This is the simulation module plan



grid_vis    : prints the grif as DOT
ltl_vis     : takes in spot::formula / Translaters Automation and prints dot
            - Vis NBA, TS and product
            - twa_product(nba, ts)

ts          : makes and returns a ts system from the graph world
            : makes a NBA from the robot
ltl         : spot wrapper: parse and translate automation type

grid_word   : grid model: bounds, obstacles, neighbors
            : eventaully add a ts_label layer

ROBOT       : capabilites, action, motion?

sim         : simulation runner; run step by step with run trace to see actions
            : should add timer (make make adjustable) so we can watch the run
            : maybe highlight the state that it is in while running though grid world (extra)
main        : just pass in a automation to see if it can be satifisfied


TO-DO list

    -  make a grid world 2X2 using the G(a -> Fb)
        - this is in test file

    - In the simulation file add a run log 
    - 


    - when there is a change you update the edge states
        - map changes -> udate TS edges
    - ts_trans.cpp
        - go though grid_world to make edges
        - ex auto -> new_edhe(0,1,p1)



    NBA -> twa_graph_ptr
        From robot
    TS  -> twa_graph_ptr
        from the graph \


NOTES:
    G: always
    F eventually
    function to take product - twa_product(a,b)

Tools:
    spot::formula
    spot::translator
    spot::twa_graph_ptr
    spot::print_dot