
#include <iostream>
#include <spot/tl/formula.hh>
#include <spot/tl/print.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twaalgos/dot.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/dot.hh>


// constructing formula
int main()
{
    // Build G(a -> Fb)

    spot::formula a   = spot::formula::ap("a");
    spot::formula Fb  = spot::formula::F(spot::formula::ap("b"));
    spot::formula f  = spot::formula::G(spot::formula::Implies(a, Fb));

    std::cout << f << '\n';

    // kindstr() prints the name of the operator
    // size() return the number of operands of the operators
    std::cout << f.kindstr() << ", " << f.size() << " children\n";
    // operator[] accesses each operand
    std::cout << "left: " << f[0]  << '\n';
    // you can also iterate over all operands using a for loop
    for (auto child: f)
    std::cout << "  * " << child << '\n';

    spot::translator trans;
    trans.set_type(spot::postprocessor::BA);        // state-based Büchi
    trans.set_pref(spot::postprocessor::Small);     // simplify
    spot::twa_graph_ptr aut = trans.run(f);         // LTL -> automaton
    spot::print_dot(std::cerr, aut);                // output automaton
    return 0;
}