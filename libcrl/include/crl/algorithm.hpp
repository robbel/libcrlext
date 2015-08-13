/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef ALGORITHM_HPP_
#define ALGORITHM_HPP_

#include <iostream>
#include <numeric>
#include "crl/function.hpp"
#include "logger.hpp"

using namespace crl;

namespace crl {

//
// Helper functions
//

/// \brief Returns a valid pointer if passed function has a tabular representation, nullptr otherwise
template<class T>
const _FDiscreteFunction<T>* is_flat(const _DiscreteFunction<T>* pf, bool known_flat) {
    return(known_flat ? static_cast<const _FDiscreteFunction<T>*>(pf) : dynamic_cast<const _FDiscreteFunction<T>*>(pf));
}

/// \brief Returns all state factors in (sorted) \a allVars that are not in cvars
/// \note Helper function for state factor selection
inline SizeVec get_state_vars(const SizeVec& allVars, const SizeVec& cvars) {
    SizeVec keepVars(cvars);
    std::sort(keepVars.begin(), keepVars.end());
    SizeVec delVars;
    std::set_difference(allVars.begin(), allVars.end(), keepVars.begin(), keepVars.end(), std::back_inserter(delVars));
    return delVars;
}

/// \brief Obtain all state factors from the \a Domain that are not in cvars
/// \note Helper function for state factor selection
inline SizeVec get_state_vars(const Domain& domain, const SizeVec& cvars) {
    SizeVec allVars = cpputil::ordered_vec<Size>(domain->getNumStateFactors());
    return get_state_vars(allVars, cvars);
}

//
// Some useful algorithms on functions
//

namespace algorithm {

/// \brief Sum the given function over its entire domain
/// \param known_flat True iff function `pf' is known to be a \a _FDiscreteFunction (optimization)
template<class T>
T sum_over_domain(const _DiscreteFunction<T>* pf, bool known_flat) {
    const _FDiscreteFunction<T>* ff = is_flat(pf, known_flat);

    if(ff) {
        return std::accumulate(ff->_sa_table->values().begin(), ff->_sa_table->values().end(), T(0));
    }
    else { // some other cases FIXME move domainSum into base class
        const _Indicator<T>* pif = dynamic_cast<const _Indicator<T>*>(pf);
        if(pif) {
            return 1;
        } else { // brute force summation over domain
            T sum = 0;
            _StateActionIncrementIterator saitr(pf->getSubdomain());
            while(saitr.hasNext()) {
                const std::tuple<State,Action>& sa = saitr.next();
                sum += pf->eval(std::get<0>(sa), std::get<1>(sa));
            }
            return sum;
        }
    }
}

/// \brief Instantiate the given function in a particular (complete w.r.t. function's subdomain) state
/// \param known_flat True iff function `pf' is known to be a \a _FDiscreteFunction (optimization)
/// \note This is a less general version of adding evidence (over some subset of variables) to the function pf
/// \return A new function with only \a Action dependencies
template<class T>
DiscreteFunction<T> instantiate(const _DiscreteFunction<T>* pf, const State& s, bool known_flat) {
  const _FDiscreteFunction<T>* of = is_flat(pf, known_flat);

  // TODO: additional optimization paths for Indicator and ConstantFn
  FDiscreteFunction<T> f = boost::make_shared<_FDiscreteFunction<T>>(pf->_domain);
  f->_action_dom = pf->getActionFactors();
  f->pack();

  if(of) {
    const Size num_actions = f->_subdomain->getNumActions();
    auto& vals  = of->_sa_table->values();
    auto& start = vals[s.getIndex()*num_actions];
    auto start_it = vals.begin() + (&start - vals.data());
    std::copy(start_it, start_it+num_actions, f->values().begin());
  }
  else { // evaluate other function exhaustively
    _ActionIncrementIterator aitr(f->getSubdomain());
    auto& vals = f->values();
    typename std::vector<T>::size_type i = 0;
    while(aitr.hasNext()) {
      vals[i++] = pf->eval(s,aitr.next());
    }
  }

  return f;
}

/// \brief Given a partial instantiation of either (local) \a State s or \a Action a, return the slice for (global) variable `i'
/// \note This is a less general version of adding evidence (over some subset of variables) to the function pf
/// \note If `i' is greater than the number of state factors, it is assumed to be an action factor
/// \return A slice with only variable `i' dependencies
template<class T>
std::vector<T> slice(const _DiscreteFunction<T>* pf, Size i, const State&s, const Action& a) {
    std::vector<T> sl;
    const Size num_state = pf->_domain->getNumStateFactors();
    FactorRange range;
    // select relevant free variable
    State rs(s);
    Action ra(a);
    RLType* prl = nullptr;
    if(i < num_state) {
        prl = &rs;
        range = pf->_domain->getStateRanges()[i];
        // convert `i' to local scope index
        i = cpputil::inverse_map<Size>(pf->getStateFactors())(i);
    }
    else {
        prl = &ra;
        i -= num_state;
        range = pf->_domain->getActionRanges()[i];
        // convert `i' to local scope index
        i = cpputil::inverse_map<Size>(pf->getActionFactors())(i);
    }

    for(Factor fa = range.getMin(); fa <= range.getMax(); fa++) {
        prl->setFactor(i, fa);
        sl.push_back(pf->eval(rs,ra));
    }

    return sl;
}

/// \brief Actual implementation for marginalization, minimization, and maximization functions
/// \tparam F The type of function returned by this operator
/// \tparam BinRefOp A type [](T& v1, T& v2) -> void
/// \see basis_gen.hpp for a template specialization for _FConjunctiveFeature
/// \see genericOp
template<class F, class T, class BinRefOp>
DiscreteFunction<T> genericOp_Shared(const _DiscreteFunction<T>* pf, SizeVec vars, bool known_flat, T init, BinRefOp binOp) {
  const _FDiscreteFunction<T>* of = is_flat(pf, known_flat);

  // TODO: optimization paths for Indicator and ConstantFn
  FDiscreteFunction<T> f = boost::make_shared<F>(pf->_domain);
  f->_state_dom = pf->getStateFactors();
  f->_action_dom = pf->getActionFactors();
  for(Size var : vars) {
      f->eraseFactor(var);
  }
  f->pack(init);
  const Size num_actions = f->_subdomain->getNumActions();
  auto& vals = f->values();

  _StateActionIncrementIterator saitr(pf->getSubdomain());
  const subdom_map s_dom(pf->getStateFactors());
  const subdom_map a_dom(pf->getActionFactors());
  const std::vector<T>* pvals = nullptr;
  typename std::vector<T>::size_type j = 0;
  if(of) {
      pvals = &of->values();
  }

  while(saitr.hasNext()) {
      const std::tuple<State,Action>& sa = saitr.next();
      const State& s = std::get<0>(sa);
      const Action& a = std::get<1>(sa);
      // apply binary operation
      T v = 0;
      v = of ? (*pvals)[j++] : (*pf)(s,a);
      State ms = f->mapState(s,s_dom);
      Action ma = f->mapAction(a,a_dom);
      const auto idx = ms.getIndex()*num_actions+ma.getIndex();
      // actual operation
      binOp(vals[idx], v);
  }

  return f;
}

/// \brief Helper class to mimic Template Function Partial Specialization of \a genericOp
/// \see http://artofsoftware.org/2012/12/20/c-template-function-partial-specialization/
/// \see basis_gen.hpp for a template specialization for _FConjunctiveFeature
/// \see genericOp
template<class F, class T, class BinRefOp>
struct genericOp_Impl {
  DiscreteFunction<T> operator()(const _DiscreteFunction<T>* pf, SizeVec vars, bool known_flat, T init, BinRefOp binOp) {
    return genericOp_Shared<F>(pf, vars, known_flat, init, binOp);
  }
};
/// \brief Actual implementation for marginalization, minimization, and maximization functions
/// \tparam F The type of function returned by this operator
/// \tparam BinRefOp A type [](T& v1, T& v2) -> void
/// \see basis_gen.hpp for a template specialization for _FConjunctiveFeature
template<class F, class T, class BinRefOp>
DiscreteFunction<T> genericOp(const _DiscreteFunction<T>* pf, SizeVec vars, bool known_flat, T init, BinRefOp binOp) {
  genericOp_Impl<F,T,BinRefOp> impl;
  return impl(pf, vars, known_flat, init, binOp);
}

/// \brief Maximize the given function over a particular state or action variable `i' (max marginalization)
/// \tparam F Template parameter passed to \a genericOp, e.g., to support different function return types
/// \param known_flat True iff function `pf' is known to be a \a _FDiscreteFunction (optimization)
/// \return A new function that is maximized over (and does not depend on) `i'
/// \note If `i' is greater than the number of state factors, it is assumed to be an action factor
template<class F, class T>
DiscreteFunction<T> maximize(const _DiscreteFunction<T>* pf, Size i, bool known_flat) {
  // implement maximization
  return algorithm::genericOp<F>(pf, {i}, known_flat, -std::numeric_limits<T>::infinity(),
                                 [](T& v1, T& v2) { if(v2 > v1) { v1 = v2; } });
}
template<class T>
DiscreteFunction<T> maximize(const _DiscreteFunction<T>* pf, Size i, bool known_flat) {
  return maximize<_FDiscreteFunction<T>>(pf, i, known_flat);
}

/// \brief Minimize the given function over a particular state or action variable `i' (min marginalization)
/// \tparam F Template parameter passed to \a genericOp, e.g., to support different function return types
/// \param known_flat True iff function `pf' is known to be a \a _FDiscreteFunction (optimization)
/// \return A new function that is minimized over (and does not depend on) `i'
/// \note If `i' is greater than the number of state factors, it is assumed to be an action factor
template<class F, class T>
DiscreteFunction<T> minimize(const _DiscreteFunction<T>* pf, Size i, bool known_flat) {
  // implement minimization
  return algorithm::genericOp<F>(pf, {i}, known_flat, std::numeric_limits<T>::infinity(),
                                 [](T& v1, T& v2) { if(v2 < v1) { v1 = v2; } });
}
template<class T>
DiscreteFunction<T> minimize(const _DiscreteFunction<T>* pf, Size i, bool known_flat) {
  return minimize<_FDiscreteFunction<T>>(pf, i, known_flat);
}

/// \brief Marginalize the given function over particular state or action variable `i' (i.e., `sum out' i)
/// \tparam F Template parameter passed to \a genericOp, e.g., to support different function return types
/// \param known_flat True iff function `pf' is known to be a \a _FDiscreteFunction (optimization)
/// \return A new function that is marginalized over (and does not depend on) `i'
/// \note If `i' is greater than the number of state factors, it is assumed to be an action factor
template<class F, class T>
DiscreteFunction<T> marginalize(const _DiscreteFunction<T>* pf, Size i, bool known_flat) {
  return marginalize(pf, {i}, known_flat);
}
template<class T>
DiscreteFunction<T> marginalize(const _DiscreteFunction<T>* pf, Size i, bool known_flat) {
  return marginalize<_FDiscreteFunction<T>>(pf, i, known_flat);
}

/// \brief Marginalize the given function over particular state or action variables `vars' (i.e., `sum out' those variables)
/// \tparam F Template parameter passed to \a genericOp, e.g., to support different function return types
/// \param known_flat True iff function `pf' is known to be a \a _FDiscreteFunction (optimization)
/// \return A new function that is marginalized over (and does not depend on) `vars'
/// \note If any entry in `vars' is greater than the number of state factors, it is assumed to be an action factor
template<class F, class T>
DiscreteFunction<T> marginalize(const _DiscreteFunction<T>* pf, SizeVec vars, bool known_flat) {
  // implement marginalization
  return algorithm::genericOp<F>(pf, vars, known_flat, 0.,
                                 [](T& v1, T& v2) { v1 += v2; });
}
template<class T>
DiscreteFunction<T> marginalize(const _DiscreteFunction<T>* pf, SizeVec vars, bool known_flat) {
  return marginalize<_FDiscreteFunction<T>>(pf, vars, known_flat);
}

/// \brief Actual implementation of \a join to combine the given set of functions into a larger one
/// \tparam F The type of function returned by this operator
/// \param binOp The operation to perform (default: sum)
/// \return A new function defined over the union of all function domains represented in `funcs'
/// \see basis_gen.hpp for a template specialization for _FConjunctiveFeature
/// \see join
template<class F, class T, class BinOp>
DiscreteFunction<T> join_Shared(cpputil::Iterator<DiscreteFunction<T>>& funcs, BinOp binOp) {
  assert(funcs.hasNext());
  const Domain& domain = funcs.next()->_domain; // same domain
  FDiscreteFunction<T> jf = boost::make_shared<F>(domain);
  // join function scopes
  funcs.reset();
  while(funcs.hasNext()) {
      jf->join(*funcs.next());
  }
  // allocate memory
  jf->pack();
  // create joint tabular values
  _StateActionIncrementIterator saitr(jf->getSubdomain());
  const subdom_map s_dom(jf->getStateFactors());
  const subdom_map a_dom(jf->getActionFactors());
  auto& vals = jf->values();
  typename std::vector<T>::size_type i = 0;
  while(saitr.hasNext()) {
      const std::tuple<State,Action>& sa = saitr.next();
      const State& s = std::get<0>(sa);
      const Action& a = std::get<1>(sa);
      T v = 0;
      funcs.reset();
      while(funcs.hasNext()) {
          const auto& f = funcs.next();
          State ms = f->mapState(s,s_dom);
          Action ma = f->mapAction(a,a_dom);
          v = binOp(v, f->eval(ms,ma));
      }
      vals[i++] = v;
  }

  return jf;
}

/// \brief Helper class to mimic Template Function Partial Specialization of \a join
/// \see http://artofsoftware.org/2012/12/20/c-template-function-partial-specialization/
/// \see basis_gen.hpp for a template specialization for _FConjunctiveFeature
/// \see join
template<class F, class T, class BinOp>
struct join_Impl {
  DiscreteFunction<T> operator()(cpputil::Iterator<DiscreteFunction<T>>& funcs, BinOp binOp) {
    return join_Shared<F>(funcs, binOp);
  }
};
/// \brief Joins the given set of functions into a larger one
/// \tparam F The type of function returned by this operator
/// \param binOp The operation to perform (default: sum)
/// \return A new function defined over the union of all function domains represented in `funcs'
template<class F, class T, class BinOp>
DiscreteFunction<T> join(cpputil::Iterator<DiscreteFunction<T>>& funcs, BinOp binOp) {
  join_Impl<F,T,BinOp> impl;
  return impl(funcs, binOp);
}
template<class T, class BinOp>
DiscreteFunction<T> join(cpputil::Iterator<DiscreteFunction<T>>& funcs, BinOp binOp) {
    return join<_FDiscreteFunction<T>>(funcs, binOp);
}
/// \brief Convenience function
/// \tparam F The type of function returned by this operator
template<class F, class T, class BinOp = decltype(std::plus<T>())>
DiscreteFunction<T> join(std::initializer_list<DiscreteFunction<T>> funcs, BinOp binOp = std::plus<T>()) {
    cpputil::ContainerIterator<DiscreteFunction<double>,std::initializer_list<DiscreteFunction<double>>> fiter(std::move(funcs));
    return join<F>(fiter, binOp);
}
template<class T, class BinOp = decltype(std::plus<T>())>
DiscreteFunction<T> join(std::initializer_list<DiscreteFunction<T>> funcs, BinOp binOp = std::plus<T>()) {
    cpputil::ContainerIterator<DiscreteFunction<double>,std::initializer_list<DiscreteFunction<double>>> fiter(std::move(funcs));
    return join<_FDiscreteFunction<T>>(fiter, binOp);
}

//
// Variable elimination
//

/// \brief Implements a heuristic for the next variable to delete
/// Greedy selection of the variable that minimizes the next joint scope
/// \return Iterator to the next best variable in the candidates ranging from \a first to \a last to eliminate
template<class InputIterator, class T>
InputIterator elimHeuristic(FunctionSet<T>& fset, InputIterator first, InputIterator last) {
  //using range = decltype(fset)::range;
  using range = typename FunctionSet<T>::range;
  auto bestIt = last;
  Size bestVal = std::numeric_limits<Size>::max();
  for(auto it = first; it != last; ++it) {
      range r = fset.getFactor(*it);
      if(!r.hasNext()) {
          continue;
      }
      _EmptyFunction<Reward> testFn(r);
//    testFn.compress();
// TODO: store best EmptyFn alongside
      Size testSc = testFn.getStateFactors().size() + testFn.getActionFactors().size() + testFn.getLiftedFactors().size();
//    Size testSc = (testFn.getStateFactors().size() + testFn.getActionFactors().size())*
//        testFn.getStateFactors().size() + testFn.getActionFactors().size();
//    for(const auto& lf : testFn.getLiftedFactors()) {
//        testSc *= lf->getStateFactors().size();
//    }
      if(testSc < bestVal) {
        bestVal = testSc;
        bestIt = it;
      }
  }
  return bestIt;
}

/// \brief Some elimination heuristics for \a variableEliminationHeur
enum class ElimHeuristic {
  NONE,
  MIN_SCOPE
};

/// \brief Runs variable elimination on the provided \a FunctionSet (the forward pass) with support for heuristic elimination order
/// The function set itself is modified in the process, as is \a mutable_elim
/// \tparam F Template parameter passed to the operator \a op, e.g., to support different function return types
/// \param[in,out] mutable_elim Set to the actual order (from first to last) implemented by the heuristc
/// \param op A function pointer denoting whether to perform max (MAP) or sum (marginalization)
/// \return A tuple of (0) the generated intermediate functions, and (1) the generated functions with empty scope
template<class F, class T, class Op = DiscreteFunction<T> (*)(const _DiscreteFunction<T>*, Size, bool)>
std::tuple<std::vector<DiscreteFunction<T>>, std::vector<DiscreteFunction<T>>>
variableEliminationHeur(FunctionSet<T>& fset, crl::SizeVec& mutable_elim, ElimHeuristic heur = ElimHeuristic::NONE, Op op = algorithm::maximize<F>) {
  LOG_DEBUG("Number of variables to eliminate: " << mutable_elim.size() << " out of " << fset.getNumFactors());

  std::list<Size> elim_list(mutable_elim.begin(),mutable_elim.end());
  mutable_elim.clear();

  std::vector<DiscreteFunction<T>> elim_cache;
  // store for functions that have reached empty scope
  std::vector<DiscreteFunction<T>> empty_fns;
  using range = typename FunctionSet<T>::range;
  while(!elim_list.empty()) {
      Size v = 0;
      if(heur == ElimHeuristic::NONE) {
          v = elim_list.front();
          elim_list.pop_front();
      } else if(heur == ElimHeuristic::MIN_SCOPE) {
          // determine next best variable to delete
          auto bestIt = elimHeuristic(fset, elim_list.begin(), elim_list.end());
          assert(bestIt != elim_list.end());
          v = *bestIt;
          LOG_DEBUG("Elim heuristic chose: " << v);
          elim_list.erase(bestIt);
      }
      mutable_elim.push_back(v);
      LOG_DEBUG("Eliminating variable " << v);
      range r = fset.getFactor(v);
      if(!r.hasNext()) {
          LOG_INFO("Variable " << v << " not eliminated. It does not exist in FunctionSet.");
          continue;
      }

      DiscreteFunction<T> esum = algorithm::join<F>(r);
      DiscreteFunction<T> emax = op(esum.get(), v, false);
      elim_cache.push_back(std::move(esum));
      fset.eraseFactor(v);
      if(emax->getSubdomain()->getNumStateFactors() == 0 && emax->getSubdomain()->getNumActionFactors() == 0) {
        LOG_DEBUG("Function reduced to empty scope");
        empty_fns.push_back(std::move(emax));
      }
      else {
        fset.insert(std::move(emax)); // continue elimination
      }
  }

  return std::make_tuple(std::move(elim_cache),std::move(empty_fns));
}

/// \brief Runs variable elimination on the provided \a FunctionSet (the forward pass) with support for heuristic elimination order
/// The function set itself is modified in the process, as is \a mutable_elim
/// \param[in,out] mutable_elim Set to the actual order (from first to last) implemented by the heuristc
/// \param op A function pointer denoting whether to perform max (MAP) or sum (marginalization)
/// \return A tuple of (0) the generated intermediate functions, and (1) the generated functions with empty scope
template<class T, class Op = DiscreteFunction<T> (*)(const _DiscreteFunction<T>*, Size, bool)>
std::tuple<std::vector<DiscreteFunction<T>>, std::vector<DiscreteFunction<T>>>
variableEliminationHeur(FunctionSet<T>& fset, crl::SizeVec& mutable_elim, ElimHeuristic heur = ElimHeuristic::NONE, Op op = algorithm::maximize<_FDiscreteFunction<T>>) {
  return variableEliminationHeur<_FDiscreteFunction<T>>(fset, mutable_elim, heur, op);
}

/// \brief Runs variable elimination on the provided \a FunctionSet (the forward pass)
/// The function set itself is modified in the process
/// \tparam F Template parameter passed to the operator \a op, e.g., to support different function return types
/// \param op A function pointer denoting whether to perform max (MAP) or sum (marginalization)
/// \return A tuple of (0) the generated intermediate functions, and (1) the generated functions with empty scope
template<class F, class T, class Op = DiscreteFunction<T> (*)(const _DiscreteFunction<T>*, Size, bool)>
std::tuple<std::vector<DiscreteFunction<T>>, std::vector<DiscreteFunction<T>>>
variableElimination(FunctionSet<T>& fset, const crl::SizeVec& elimination_order, Op op = algorithm::maximize<F>) {
  SizeVec mutable_elim(elimination_order);
  return variableEliminationHeur<F>(fset, mutable_elim, ElimHeuristic::NONE, op);
}
/// \brief Runs variable elimination on the provided \a FunctionSet (the forward pass)
/// The function set itself is modified in the process
/// \param op A function pointer denoting whether to perform max (MAP) or sum (marginalization)
/// \return A tuple of (0) the generated intermediate functions, and (1) the generated functions with empty scope
template<class T, class Op = DiscreteFunction<T> (*)(const _DiscreteFunction<T>*, Size, bool)>
std::tuple<std::vector<DiscreteFunction<T>>, std::vector<DiscreteFunction<T>>>
variableElimination(FunctionSet<T>& fset, const crl::SizeVec& elimination_order, Op op = algorithm::maximize<_FDiscreteFunction<T>>) {
  return variableElimination<_FDiscreteFunction<T>>(fset, elimination_order, op);
}

/// \brief Runs variable elimination on the provided \a FunctionSet and computes the argmax (in a backward pass)
/// \return The maximizing variable setting and the (maximal) value
template<class T>
std::tuple<Action,T> argVariableElimination(FunctionSet<T>& F, const crl::SizeVec& elimination_order) {
  // Make sure we can eliminate all variables in function set
  assert(elimination_order.size() >= F.getNumFactors());
  // run variable elimination
  auto retFns = variableElimination(F, elimination_order);
  const std::vector<DiscreteFunction<T>>& elim_cache = std::get<0>(retFns);
  const std::vector<DiscreteFunction<T>>& empty_fns  = std::get<1>(retFns);

  if(!F.empty()) {
      throw cpputil::InvalidException("Functions remain in function set: argVariableElimination failed");
  }

  // sum over all empty functions defines the maximum value
  T maxVal = std::accumulate(empty_fns.begin(), empty_fns.end(), T(0), [](T store, const DiscreteFunction<T>& f) {
      return store + f->eval(State(),Action()); });

  if(elim_cache.empty()) {
      return std::make_tuple(Action(),maxVal);
  }

  // argmax in reverse order
  assert(elim_cache.size() == elimination_order.size());
  const Domain& domain = elim_cache.front()->_domain;
  const RangeVec ranges = domain->getActionRanges();
  const Size num_state = domain->getNumStateFactors();

  Action retMax(domain); // the globally maximizing joint action
  auto fit = elim_cache.rbegin(); // function iterator
  for (auto rit = elimination_order.rbegin(); rit != elimination_order.rend(); ++rit, ++fit) {
      Size v = *rit;
      assert(v >= num_state); // eliminating an action variable
      Action ma((*fit)->mapAction(retMax)); // map to local function's domain
      const std::vector<T>& slice = algorithm::slice(fit->get(), v, State(), ma);
      auto optIt = std::max_element(slice.begin(), slice.end()); // yields the /first/ maximizing action
      assert(optIt != slice.end()); // empty range check
      auto offset = optIt-slice.begin();
      // update maximizing action
      retMax.setFactor(v-num_state, ranges[v-num_state].getMin()+offset);
  }

  return std::make_tuple(retMax, maxVal);
}

} // namespace algorithm

} // namespace crl

#endif /*ALGORITHM_HPP_*/
