/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include "crl/alp.hpp"

using namespace std;

namespace crl {

} // namespace crl

using namespace crl;

int main() {
  srand(0);

  Domain domain = boost::make_shared<_Domain>();
  domain->addStateFactor(0, 299, "first_state"); // 300 states
  domain->addActionFactor(0, 4, "first_agent");  // 5 actions
  domain->setRewardRange(-1, 0);

  State s(domain, 1);
  State s2(domain,2);
  Action a(domain,0);

  Indicator I = boost::make_shared<_Indicator>(domain, domain, s);
  assert((*I)(s) && !(*I)(s2));

  DBN dbn;
  dbn->T(s,a,s,identity_map,identity_map,identity_map);

  _Backprojection<double> b(domain, dbn, I);

//  Backprojection<int> b = boost::make_shared<_Backprojection<int>>(dbn,I);
}
