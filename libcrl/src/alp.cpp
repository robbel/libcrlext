/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <numeric>
#include "crl/alp.hpp"
#include "crl/alp_lpsolve.hpp"
#include "logger.hpp"

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

Reward _FactoredValueFunction::getV(const State &js) const {
  assert(_weight.size() == _basis.size());
  double res = std::inner_product(_weight.begin(), _weight.end(), _basis.begin(), 0., std::plus<double>(),
      [&js](double w, const DiscreteFunction<Reward>& b) {
          return w * b->eval(b->mapState(js));
  });
  return res;
}

Action _FactoredValueFunction::getBestAction(const State& js, const SizeVec& elimination_order) const {
  // todo

  return Action();
}

//
// ALPPlanner implementation
//

void _ALPPlanner::precompute() {
    // compute backprojections and state relevance weights
    for(const DiscreteFunction<Reward>& h : _value_fn->getBasis()) {
        const _ConstantFn<Reward>* cfn = dynamic_cast<const _ConstantFn<Reward>*>(h.get());
        if(cfn) {
            // Backprojection of a constant function is the identical function
            ConstantFn<Reward> C = boost::make_shared<_ConstantFn<Reward>>(*cfn);
            (*C) *= _gamma;
            (*C) -= *cfn;
            _C_set.push_back(std::move(C));
        }
        else {
            // Full backprojection through DBN
            Backprojection<Reward> B = boost::make_shared<_Backprojection<Reward>>(_domain, _fmdp->T(), h);
            B->cache();
            (*B) *= _gamma;
            (*B) -= h.get(); // FIXME in indicator case, h is not a `flat' function, implement efficient sparse multiplication
            _C_set.push_back(std::move(B));
        }
        // compute factored state relevance weights assuming uniform state likelihoods
        const Size dom_size = h->getSubdomain()->size();
        Reward r_sum = algorithm::sum_over_domain(h.get(), false);
        _alpha.push_back(r_sum/dom_size);
    }
}

int _ALPPlanner::plan() {
    precompute();

    // build the lp constraints from the target functions above
    // create a basic elimination order (sorted state variables, followed by sorted action variables)
    SizeVec elim_order = cpputil::ordered_vec<Size>(_domain->getNumStateFactors() + _domain->getNumActionFactors());

    lpsolve::_LP lp(_domain);
    int res = lp.generateLP(_C_set, _fmdp->getLRFs(), _alpha, elim_order);
    if(res != 0) {
      LOG_ERROR("generateLP() error code: " << res);
      return 1;
    }

    res = lp.solve(_value_fn.get());
    if(res != 0) {
      LOG_ERROR("solve() error code: " << res);
      return 2;
    }

    return 0; // success
}

int _BLPPlanner::plan() {
    precompute();

    lpsolve::_LP lp(_domain);
    int res = lp.generateBLP(_C_set, _fmdp->getLRFs(), _alpha);
    if(res != 0) {
      LOG_ERROR("generateBLP() error code: " << res);
      return 1;
    }

    res = lp.solve(_value_fn.get());
    if(res != 0) {
      LOG_ERROR("solve() error code: " << res);
      return 2;
    }

    return 0; // success
}

} // namespace crl
