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
const Size OptBEBFScore::DEFAULT_MAX_SCOPE = 12;

std::ostream& operator<<(std::ostream &os, const Conjunction& conj) {
  os << "Conj({";
  for(auto h : conj.getBaseFeatures()) {
      os << "h" << h << "^";
  }
  os << "}";
  return os;
}

double EpsilonScore::score(const DiscreteFunction<Reward>& basis) const {
  assert(basis->getActionFactors().empty());
  // remove variables to reach basis' domain
  const SizeVec elim_order = get_state_vars(_domain, basis->getStateFactors());

  // evaluate max over basis function
  auto facfn = algorithm::factoredBellmanResidual(_domain, _value_fn, elim_order, algorithm::maximize);
  double maxVal = algorithm::evalOpOverBasis(basis.get(), facfn, false, -std::numeric_limits<double>::infinity(),
                                             [](Reward& v1, Reward& v2) { if(v2 > v1) { v1 = v2; } });
  // evaluate min over basis function
  facfn = algorithm::factoredBellmanResidual(_domain, _value_fn, elim_order, algorithm::minimize);
  double minVal = algorithm::evalOpOverBasis(basis.get(), facfn, false, std::numeric_limits<double>::infinity(),
                                             [](Reward& v1, Reward& v2) { if(v2 < v1) { v1 = v2; } });
  minVal = minVal < 0. ? 0. : minVal;

  double range = 0.5*(maxVal - minVal);
  LOG_DEBUG("Epsilon (before normalization): " << range);

  // TODO: this is just temporary coverage metric
  Size cov = algorithm::basisCoverage(_domain, basis.get());
  if(cov == 0) {
      return -std::numeric_limits<double>::infinity();
  }

  return range / sqrt(static_cast<double>(cov));
}

double AbsoluteReductionScore::score(const DiscreteFunction<Reward>& basis) const {
  assert(basis->getActionFactors().empty());
  // remove variables to reach basis' domain
  const SizeVec elim_order = get_state_vars(_domain, basis->getStateFactors());

  // evaluate max over basis function
  auto facfn = algorithm::factoredBellmanResidual(_domain, _value_fn, elim_order, algorithm::maximize);
  double maxVal = algorithm::evalOpOverBasis(basis.get(), facfn, false, -std::numeric_limits<double>::infinity(),
                                             [](Reward& v1, Reward& v2) { if(v2 > v1) { v1 = v2; } });
  // evaluate min over basis function
  facfn = algorithm::factoredBellmanResidual(_domain, _value_fn, elim_order, algorithm::minimize);
  double minVal = algorithm::evalOpOverBasis(basis.get(), facfn, false, std::numeric_limits<double>::infinity(),
                                             [](Reward& v1, Reward& v2) { if(v2 < v1) { v1 = v2; } });
  minVal = minVal < 0. ? 0. : minVal;

  double red = 0.5*(maxVal + minVal);
  LOG_DEBUG("Absolute reduction (before normalization): " << red);

  // TODO: this is just temporary coverage metric
  Size cov = algorithm::basisCoverage(_domain, basis.get());
  if(cov == 0) {
      return -std::numeric_limits<double>::infinity();
  }

  return red / sqrt(static_cast<double>(cov));
}

void BEBFScore::initialize() {
  // set once before the (complete) scoring run over basis functions
  _maxQ = algorithm::factoredBellmanFunctionals(_domain, _value_fn).getFunctions();
}

double BEBFScore::score(const DiscreteFunction<Reward>& basis) const {
  assert(basis->getActionFactors().empty());

  // evaluate marginal under basis
  auto facfn = algorithm::factoredBellmanMarginal(_domain, basis->getStateFactors(), _maxQ);
  double marVal = algorithm::evalOpOverBasis(basis.get(), facfn, false, 0.,
                                             [](Reward& v1, Reward& v2) { v1 += v2; });
  LOG_DEBUG("Marginal under feature: " << marVal);

  // TODO: assert no overflow

  Size cov = algorithm::basisCoverage(_domain, basis.get());
  if(cov == 0) {
      return -std::numeric_limits<double>::infinity();
  }
  return marVal / sqrt(static_cast<double>(cov));
}

void OptBEBFScore::initialize() {
  // set once before the (complete) scoring run over basis functions
  // we additionally keep track of the `action-connectivity' property of the maxQ-function for efficient partition size computations
  _maxQ = algorithm::factoredBellmanFunctionals<_FConjunctiveFeature<Reward>>(_domain, _value_fn).getFunctions();
}

double OptBEBFScore::score(const DiscreteFunction<Reward>& basis) const {
  _Backprojection<Reward> B(_domain, _dbn, basis);
  const SizeVec& adom = B.getActionFactors();

  // find the size of the `action-connected' partition
  vector<bool> active(_domain->getNumStateFactors(), false);
  for(Size v : B.getStateFactors()) {
    active[v] = true;
  }
  for(const auto& fn : _maxQ) {
      const Conjunction* cf = dynamic_cast<const Conjunction*>(fn.get());
      const SizeVec& fadom = cf ? cf->getBaseFeatures() : fn->getActionFactors();
      if(cpputil::has_intersection(adom.begin(), adom.end(), fadom.begin(), fadom.end())) {
          for(Size v : fn->getStateFactors()) {
            active[v] = true;
          }
      }
  }
  Size maxpt = std::count(active.begin(), active.end(), true);
#if !NDEBUG
  const Conjunction* cpt = dynamic_cast<Conjunction*>(basis.get());
  LOG_DEBUG("Candidate fn " << *cpt << ": " << *basis << " (bp: " << B << ") `action-connects' " << maxpt << " state factors.");
#endif

  if(maxpt > _max_scope) {
    return -std::numeric_limits<double>::infinity();
  }

  return BEBFScore::score(std::move(basis));
}

} // namespace crl
