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

double EpsilonScore::score(const _DiscreteFunction<Reward>* basis) const {
  assert(basis->getActionFactors().empty());
  // remove variables to reach basis' domain
  const SizeVec elim_order = get_state_vars(_domain, basis->getStateFactors());

  // evaluate max over basis function
  auto facfn = algorithm::factoredBellmanResidual(_domain, _value_fn, elim_order, algorithm::maximize);
  double maxVal = algorithm::evalOpOverBasis(basis, facfn, false, -std::numeric_limits<double>::infinity(),
                                             [](Reward& v1, Reward& v2) { if(v2 > v1) { v1 = v2; } });
  // evaluate min over basis function
  facfn = algorithm::factoredBellmanResidual(_domain, _value_fn, elim_order, algorithm::minimize);
  double minVal = algorithm::evalOpOverBasis(basis, facfn, false, std::numeric_limits<double>::infinity(),
                                             [](Reward& v1, Reward& v2) { if(v2 < v1) { v1 = v2; } });
  minVal = minVal < 0. ? 0. : minVal;

  double range = maxVal - minVal;
  LOG_DEBUG("Feature range: " << range);
  return range;
}

double BEBFScore::score(const _DiscreteFunction<Reward> *basis) const {
  assert(basis->getActionFactors().empty());

  // evaluate marginal under basis
  auto facfn = algorithm::factoredBellmanMarginal(_domain, basis->getStateFactors(), _value_fn);
  double marVal = algorithm::evalOpOverBasis(basis, facfn, false, 0.,
                                             [](Reward& v1, Reward& v2) { v1 += v2; });
  LOG_DEBUG("Marginal under feature: " << marVal);

  // TODO: include coverage of a feature (size of dom where it's active)

  return marVal;
}

} // namespace crl
