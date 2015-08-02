/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <iostream>
#include <cmath>
#include "crl/basis_gen.hpp"

using namespace std;

namespace crl {

const std::size_t Conjunction::EMPTY_HASH = 0;

std::ostream& operator<<(std::ostream &os, const Conjunction& conj) {
  os << "Conj({";
  for(auto h : conj.getBaseFeatures()) {
      os << "h" << h << "^";
  }
  os << "}";
  return os;
}

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
  if(minVal < 0.) {
      LOG_DEBUG("[EpsilonScore::score]: minVal " << minVal);
  }
  minVal = minVal < 0. ? 0. : minVal;

  double range = 0.5*(maxVal - minVal);
  LOG_DEBUG("Epsilon (before normalization): " << range);

  // TODO: this is just temporary coverage metric
  Size cov = algorithm::basisCoverage(_domain, basis);
  if(cov == 0) {
      return -std::numeric_limits<double>::infinity();
  }

  return range / sqrt(static_cast<double>(cov));
}

double AbsoluteReductionScore::score(const _DiscreteFunction<Reward>* basis) const {
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
  if(minVal < 0.) {
      LOG_DEBUG("[AbsoluteReductionScore::score]: minVal " << minVal);
  }
  minVal = minVal < 0. ? 0. : minVal;

  double red = 0.5*(maxVal + minVal);
  LOG_DEBUG("Absolute reduction (before normalization): " << red);

  // TODO: this is just temporary coverage metric
  Size cov = algorithm::basisCoverage(_domain, basis);
  if(cov == 0) {
      return -std::numeric_limits<double>::infinity();
  }

  return red / sqrt(static_cast<double>(cov));
}

double BEBFScore::score(const _DiscreteFunction<Reward> *basis) const {
  assert(basis->getActionFactors().empty());

  // evaluate marginal under basis
  auto facfn = algorithm::factoredBellmanMarginal(_domain, basis->getStateFactors(), _value_fn);
  double marVal = algorithm::evalOpOverBasis(basis, facfn, false, 0.,
                                             [](Reward& v1, Reward& v2) { v1 += v2; });
  LOG_DEBUG("Marginal under feature: " << marVal);

  // TODO: assert no overflow

  Size cov = algorithm::basisCoverage(_domain, basis);
  if(cov == 0) {
      return -std::numeric_limits<double>::infinity();
  }
  return marVal / sqrt(static_cast<double>(cov));
}

} // namespace crl
