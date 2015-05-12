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
#if HAS_GUROBI
#include "crl/alp_gurobi.hpp"
#endif
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

std::tuple<Action,Reward> _FactoredValueFunction::getBestAction(const State& js, const SizeVec& elimination_order) {
    assert(_backprojection.size() == _basis.size());
    assert(!_lrfs.empty());

    // Discount stored backprojections once to form local Q functions
    if(!_bp_discounted) {
        discount();
    }

    // run argVariableElimination over joint action space
    // store for functions that have reached empty scope
    std::vector<DiscreteFunction<Reward>> empty_fns;
    // functions are instantiated in state `js'
    FunctionSet<Reward> F(_domain);
    for(auto& bp : _backprojection) {
        State ms = bp->mapState(js);
        DiscreteFunction<Reward> f = algorithm::instantiate(bp.get(), ms, false);
        if(f->getSubdomain()->getNumStateFactors() == 0 && f->getSubdomain()->getNumActionFactors() == 0) {
          empty_fns.push_back(std::move(f));
        }
        else {
            F.insert(std::move(f));
        }
    }
    for(auto& b : _lrfs) {
        State ms = b->mapState(js);
        DiscreteFunction<Reward> f = algorithm::instantiate(b.get(), ms, false);
        if(f->getSubdomain()->getNumStateFactors() == 0 && f->getSubdomain()->getNumActionFactors() == 0) {
          empty_fns.push_back(std::move(f));
        }
        else {
            F.insert(std::move(f));
        }
    }

    tuple<Action,Reward> tpl = algorithm::argVariableElimination(F, elimination_order);
    // add value of empty-scope functions to final result
    std::get<1>(tpl) = std::accumulate(empty_fns.begin(), empty_fns.end(), std::get<1>(tpl),
        [](Reward store, const DiscreteFunction<Reward>& f) { return store + f->eval(State(),Action()); });

    if(!std::get<0>(tpl)) { // empty action (i.e, when backprojections and reward functions have no action dependencies)
        std::get<0>(tpl) = Action(_domain,0); // simply choose first one
    }

    return tpl;
}

Reward _FactoredValueFunction::getQ(const State& js, const Action& ja) {
    assert(_backprojection.size() == _basis.size());
    assert(!_lrfs.empty());
    assert(js);

    // Discount stored backprojections once to form local Q functions
    if(!_bp_discounted) {
        discount();
    }

    double ret = 0.;
    Action ma(ja);
    for(auto& bp : _backprojection) {
        State ms = bp->mapState(js);
        if(ja) { // backprojections may have empty action scope
          ma = bp->mapAction(ja);
        }
        ret += bp->eval(ms,ma);
    }
    for(auto& b : _lrfs) {
        State ms = b->mapState(js);
        if(ja) {
          ma = b->mapAction(ja);
        }
        ret += b->eval(ms,ma);
    }

    return static_cast<Reward>(ret);
}

void _FactoredValueFunction::discount() {
    // Discount stored backprojections once to form part of local Q functions
    std::vector<double>::size_type i = 0;
    for(auto& bp : _backprojection) {
        _ConstantFn<Reward>* cfn = dynamic_cast<_ConstantFn<Reward>*>(bp.get());
        if(cfn) {
            (*cfn) *= _weight[i++];
        }
        else {
            _FDiscreteFunction<Reward>* fd = static_cast<_FDiscreteFunction<Reward>*>(bp.get());
            (*fd) *= _weight[i++];
        }
    }
    _bp_discounted = true;
}

//
// ALPPlanner implementation
//

void _ALPPlanner::precompute() {
    // clear previously computed ones
    _alpha.clear();
    _C_set.clear();
    // compute backprojections and state relevance weights
    for(const DiscreteFunction<Reward>& h : _value_fn->getBasis()) {
        const _ConstantFn<Reward>* cfn = dynamic_cast<const _ConstantFn<Reward>*>(h.get());
        if(cfn) {
            // Backprojection of a constant function is the identical function
            ConstantFn<Reward> C = boost::make_shared<_ConstantFn<Reward>>(*cfn);
            (*C) *= _gamma;
            // store a (deep) copy of the backprojected basis in the factored value function
            _value_fn->addBackprojection(boost::make_shared<_ConstantFn<Reward>>(*C));
            (*C) -= *cfn;
            _C_set.push_back(std::move(C));
        }
        else {
            // Full backprojection through DBN
            Backprojection<Reward> B = boost::make_shared<_Backprojection<Reward>>(_domain, _fmdp->T(), h);
            B->cache();
            (*B) *= _gamma;
//#if !NDEBUG
//            LOG_DEBUG("\\gamma * Backprojection:");
//            _StateActionIncrementIterator saitr(B->getSubdomain());
//            while(saitr.hasNext()) {
//                const std::tuple<State,Action>& sa = saitr.next();
//                LOG_DEBUG(std::get<0>(sa) << " " << std::get<1>(sa) << ": " << B->eval(std::get<0>(sa), std::get<1>(sa)));
//            }
//#endif
            // store a (deep) copy of the backprojected basis in the factored value function
            _value_fn->addBackprojection(boost::make_shared<_Backprojection<Reward>>(*B));
            (*B) -= h.get(); // FIXME in indicator case, h is not a `flat' function, implement efficient sparse multiplication
//#if !NDEBUG
//            LOG_DEBUG("\\gamma * Backprojection - h(x):");
//            saitr.reset();
//            while(saitr.hasNext()) {
//                const std::tuple<State,Action>& sa = saitr.next();
//                LOG_DEBUG(std::get<0>(sa) << " " << std::get<1>(sa) << ": " << B->eval(std::get<0>(sa), std::get<1>(sa)));
//            }
//#endif
            _C_set.push_back(std::move(B));
        }
        // compute factored state relevance weights assuming uniform state likelihoods
        const Size dom_size = h->getSubdomain()->size();
        Reward r_sum = algorithm::sum_over_domain(h.get(), false); // FIXME for arbitrary ConstantFn! Consider internally T=double
        _alpha.push_back(r_sum/dom_size);
    }
    // make lrfs available for local Q-function computations in factored value function
    _value_fn->setLRFs(_fmdp->getLRFs());
}

int _ALPPlanner::plan() {
    precompute();

    // build the lp constraints from the target functions above
    // create a basic elimination order (sorted state variables, followed by sorted action variables)
    SizeVec elim_order = cpputil::ordered_vec<Size>(_domain->getNumStateFactors() + _domain->getNumActionFactors());

    long start_time = cpputil::time_in_milli();
#if HAS_GUROBI
    gurobi::_LP lp(_domain);
#else
    lpsolve::_LP lp(_domain);
#endif
    int res = lp.generateLP(_C_set, _fmdp->getLRFs(), _alpha, elim_order);
    if(res != 0) {
      LOG_ERROR("generateLP() error code: " << res);
      return 1;
    }
    long end_time = cpputil::time_in_milli();
    LOG_INFO("generateLP() returned after " << end_time - start_time << "ms");

    res = lp.solve(_value_fn.get());
    if(res != 0) {
      LOG_ERROR("solve() error code: " << res);
      return 2;
    }

    return 0; // success
}

int _BLPPlanner::plan() {
    precompute();

    long start_time = cpputil::time_in_milli();
#if HAS_GUROBI
    gurobi::_LP lp(_domain);
#else
    lpsolve::_LP lp(_domain);
#endif
    int res = lp.generateBLP(_C_set, _fmdp->getLRFs(), _alpha);
    if(res != 0) {
      LOG_ERROR("generateBLP() error code: " << res);
      return 1;
    }
    long end_time = cpputil::time_in_milli();
    LOG_INFO("generateBLP() returned after " << end_time - start_time << "ms");

    res = lp.solve(_value_fn.get());
    if(res != 0) {
      LOG_ERROR("solve() error code: " << res);
      return 2;
    }

    return 0; // success
}
#if 0
int _VLPPlanner::plan() {
    //precompute(); // does not use _C_set (backprojections)
    _alpha.clear();
    _C_set.clear();
    for(const auto& h : _value_fn->getBasis()) {
        // compute factored state relevance weights assuming uniform state likelihoods
        const Size dom_size = h->getSubdomain()->size();
        Reward r_sum = algorithm::sum_over_domain(h.get(), false); // FIXME for arbitrary ConstantFn! Consider internally T=double
        _alpha.push_back(r_sum/dom_size);
    }
    // make lrfs available for local Q-function computations in factored value function
    _value_fn->setLRFs(_fmdp->getLRFs());

    long start_time = cpputil::time_in_milli();
#if HAS_GUROBI
    gurobi::_LP lp(_domain);
#else
    lpsolve::_LP lp(_domain);
#endif
    int res = lp.generateVLP(_value_fn->getBasis(), _fmdp->getLRFs(), _alpha, _fmdp->T(), _gamma);
    if(res != 0) {
      LOG_ERROR("generateVLP() error code: " << res);
      return 1;
    }
    long end_time = cpputil::time_in_milli();
    LOG_INFO("generateVLP() returned after " << end_time - start_time << "ms");

    res = lp.solve(_value_fn.get());
    if(res != 0) {
      LOG_ERROR("solve() error code: " << res);
      return 2;
    }

    return 0; // success
}
#endif
} // namespace crl
