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
    // compute backprojections and state relevance weights
    for(const DiscreteFunction<Reward>& h : _value_fn->getBasis()) {
        Backprojection<Reward> B = boost::make_shared<_Backprojection<Reward>>(_domain, _fmdp->T(), h);
        B->cache();
        (*B) *= _gamma;
        (*B) -= h.get(); // FIXME in indicator case, h is not a `flat' function, implement efficient sparse multiplication
        _C_set.push_back(std::move(B));
        // compute factored state relevance weights assuming uniform state likelihoods
        const Size dom_size = h->getSubdomain()->getNumStates() * h->getSubdomain()->getNumActions();
        Reward r_sum = algorithm::sum_over_domain(h.get(), false);
        _alpha.push_back(r_sum/dom_size);
    }

    // build the lp constraints from the target functions above
    // create a basic elimination order (sorted state variables, followed by sorted action variables)
    SizeVec elim_order = cpputil::ordered_vec<Size>(_domain->getNumStateFactors() + _domain->getNumActionFactors());

    lpsolve::_LP lp(_domain);
    if(!lp.generateLP(_C_set, _fmdp->getLRFs(), _alpha, elim_order)) {
      return 1;
    }

    if(!lp.solve(_value_fn.get())) {
      return 2;
    }

    // w values in FactoredValueFunction have been computed successfully
    // todo..

    return 0;
}

} // namespace crl
