
#include <iostream>
#include <spot/tl/formula.hh>
#include <spot/tl/print.hh>

// constructing formula
int main()
{
  // Build FGa -> (GFb & GFc)
  spot::formula fga = spot::formula::F(spot::formula::G(spot::formula::ap("a")));
  spot::formula gfb = spot::formula::G(spot::formula::F(spot::formula::ap("b")));
  spot::formula gfc = spot::formula::G(spot::formula::F(spot::formula::ap("c")));
  spot::formula f = spot::formula::Implies(fga, spot::formula::And({gfb, gfc}));

  std::cout << f << '\n';

  // kindstr() prints the name of the operator
  // size() return the number of operands of the operators
  std::cout << f.kindstr() << ", " << f.size() << " children\n";
  // operator[] accesses each operand
  std::cout << "left: " << f[0] << ", right: " << f[1] << '\n';
  // you can also iterate over all operands using a for loop
  for (auto child: f)
    std::cout << "  * " << child << '\n';
  // the type of the operator can be accessed with kind(), which
  // return an element of the spot::op enum.
  std::cout << f[1][0]
            << (f[1][0].kind() == spot::op::F ? " is F\n" : " is not F\n");
  // however because writing f.kind() == spot::op::XXX is quite common, there
  // is also a is() shortcut:
  std::cout << f[1][1]
            << (f[1][1].is(spot::op::G) ? " is G\n" : " is not G\n");
  return 0;
}