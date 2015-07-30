/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <iostream>
#include "crl/basis_gen.hpp"

using namespace std;

namespace crl {

namespace algorithm {

double scoreBasis(const Domain& domain, const DiscreteFunction<Reward>& basis, const FactoredValueFunction& fval) {
  assert(basis->getActionFactors().empty());
  // remove variables (here: under `max') to reach basis' domain
  const SizeVec elim_order = get_state_vars(domain, basis->getStateFactors());
  const auto facfn = factoredBellmanResidual(domain, fval, elim_order, algorithm::maximize);

  // evaluate basis function
  double v = evalOpBasis(basis.get(), facfn, false, -std::numeric_limits<double>::infinity(),
                         [](Reward& v1, Reward& v2) { if(v2 > v1) { v1 = v2; } });
  return v;
}

} // namespace algorithm

} // namespace crl
