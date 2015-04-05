/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include "crl/alp.hpp"
#include "crl/alp_lpsolve.hpp"

using namespace std;

namespace crl {

//
// FactoredValueFunction implementation
//

void _FactoredValueFunction::addBasisFunction(DiscreteFunction<Reward> h, double weight) {
  _basis.push_back(std::move(h));
  _weight.push_back(weight);
}

void _FactoredValueFunction::setWeight(Size i, double weight) {
  if(i >= _basis.size()) {
      throw cpputil::IndexException(i, _basis.size(), "Basis function not available");
  }
  _weight[i] = weight;
}

Reward _FactoredValueFunction::getV(const State &s) const {
  assert(_weight.size() == _basis.size());

  // todo

  return 0.;
}

Action _FactoredValueFunction::getBestAction(const State& js) const {

  // todo

  return Action();
}

Action _FactoredValueFunction::getBestAction(const State& js, const SizeVec& elimination_order) const {

  // todo

  return Action();
}

//
// ALPPlanner implementation
//

int _ALPPlanner::plan() {
  lpsolve::lp_exp();
  int res = lpsolve::lp_demo();
  return res;
}

} // namespace crl
