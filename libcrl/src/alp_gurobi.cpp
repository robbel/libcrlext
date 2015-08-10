/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include "crl/alp_gurobi.hpp"
#include "logger.hpp"

using namespace std;
using namespace crl;

namespace {

/// \brief Fills rl2 with corresponding entries from rl1, if available (as determined by the respective \a subdom_map)
/// rl1 and rl2 must have identical type (i.e., either \a State or \a Action)
/// \see _LP::generateLiftedLP
void buildRLType(const RLType& rl1, const subdom_map& s1, RLType& rl2, const subdom_map& s2) {
  const auto& fullmap = s2.map();
  const auto& submap = s1.map();
  for(auto kv : fullmap) {
      auto search = submap.find(kv.first);
      //kv.first: the global factor Id
      if(search != submap.end()) { // s1's subdomain includes the element
        //kv.second: the local mapping in the given rl2
        rl2.setFactor(kv.second, rl1.getFactor(search->second));
      }
  }
}

/// \brief Return the number of `enabled' (i.e., != 0) factors in \a State s that pertain to the given variables in \a range.
/// \note In addition to the variables included in \a range, the variable `v' with value `val' is considered.
/// \see _LP::generateLiftedLP
typedef std::pair<std::size_t,Size> HashSizePair;
typedef std::unordered_multimap<std::size_t,Size> HashSizeMap;
inline Factor enabledCount(const RLType& s, const subdom_map& sl_dom, Size v, Factor val, const std::pair<HashSizeMap::const_iterator, HashSizeMap::const_iterator>& range) {
  Factor ret = 0;
  std::for_each(range.first, range.second, [&](const HashSizePair& hsp) {
    bool enabled = hsp.second != v ? (s.getFactor(sl_dom(hsp.second)) != 0) : (val != 0);
    ret += static_cast<Factor>(enabled);
  });
  return ret;
}

} // anonymous ns

namespace gurobi {

void _LP::generateObjective(std::string name, const std::vector<double>& alpha) {
  // the variable store
  _wvars.clear();
  _wvars.reserve(alpha.size());

  // create the lp
  _env = boost::make_shared<GRBEnv>();
  _lp  = boost::make_shared<GRBModel>(*_env);
  _lp->set(GRB_StringAttr_ModelName, name);

  // add the weight variables with [-inf,inf] bounds along with the objective coefficients
  for(vector<double>::size_type i = 0; i < alpha.size(); i++) {
    GRBVar v = _lp->addVar(-GRB_INFINITY, GRB_INFINITY, alpha[i], GRB_CONTINUOUS, "w"+to_string(i));
    _wvars.push_back(std::move(v));
  }

  // The objective is to minimize the costs
  _lp->set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
  _lp->update();
  LOG_INFO("Objective: " << _lp->getObjective());
}

std::tuple<std::unordered_map<const _DiscreteFunction<Reward>*, int>, std::vector<DiscreteFunction<Reward>>>
_LP::generateObjective(std::string name, const crl::RFunctionVec& C, const crl::RFunctionVec& b, const std::vector<double>& alpha) {
  //assert(C.size() == alpha.size());

  generateObjective(name, alpha);
  // the function set for functions with non-empty scope (available for, e.g., variable elimination)
  _F.clear();

  //
  // generate equality constraints to abstract away basis (C) functions
  //

  std::unordered_map<const _DiscreteFunction<Reward>*, int> var_offset;
  // store for functions that have reached empty scope (those are used in last constraint)
  std::vector<DiscreteFunction<Reward>> empty_fns;

  int var = alpha.size(); // the offset at which to insert more variables
  int wi = 0; // corresponding to active w_i variable
  for(const auto& f : C) {
#if !NDEBUG
      const auto& p =
#endif
      var_offset.insert({f.get(),var});
      assert(p.second);
      int cc = 0;
      _StateActionIncrementIterator saitr(f->getSubdomain());
      while(saitr.hasNext()) {
          const std::tuple<State,Action>& sa = saitr.next();
          const State& s = std::get<0>(sa);
          const Action& a = std::get<1>(sa);
          // add variable
          GRBVar v = _lp->addVar(-GRB_INFINITY, GRB_INFINITY, 0., GRB_CONTINUOUS, "C"+to_string(wi)+"_"+to_string(cc));
          _lp->update();
          GRBLinExpr lhs = _wvars[wi] * (*f)(s,a);
          addConstraint(lhs, GRB_EQUAL, v);
          var++;
          cc++;
      }
      // test for constant basis
      if(f->getSubdomain()->getNumStateFactors() == 0 && f->getSubdomain()->getNumActionFactors() == 0) {
          LOG_DEBUG("Found constant basis function with empty scope");
          empty_fns.push_back(std::move(f));
      }
      else {
          _F.insert(std::move(f)); // store function in set for variable elimination
      }
      wi++;
  }

  //
  // generate equality constraints to abstract away target (b) functions
  //

  int bb = 0;
  for(const auto& f : b) {
#if !NDEBUG
      const auto& p =
#endif
      var_offset.insert({f.get(),var});
      assert(p.second);
      int bv = 0;
      _StateActionIncrementIterator saitr(f->getSubdomain());
      while(saitr.hasNext()) {
          const std::tuple<State,Action>& sa = saitr.next();
          const State& s = std::get<0>(sa);
          const Action& a = std::get<1>(sa);
          // add variable
          GRBVar v = _lp->addVar(-GRB_INFINITY, GRB_INFINITY, 0., GRB_CONTINUOUS, "b"+to_string(bb)+"_"+to_string(bv));
          _lp->update();
          addConstraint(v, GRB_EQUAL, (*f)(s,a));
          var++;
          bv++;
      }
      _F.insert(std::move(f));
      bb++;
  }

  return std::make_tuple(std::move(var_offset),std::move(empty_fns));
}

void _LP::addStateActionConstraint(const State& s, const Action& a, const RFunctionVec& C, const RFunctionVec& b) {
  static vector<double> coeff(_wvars.size()); // coefficient cache

  // write out one constraint corresponding to s, a
  GRBLinExpr lhs = 0;
  vector<double>::size_type i = 0;
  for(const auto& f : C) {
      State ms = f->mapState(s);
      Action ma = f->mapAction(a);
      coeff[i++] = (*f)(ms,ma);
  }
  lhs.addTerms(coeff.data(), _wvars.data(), _wvars.size());
  double sum = 0.;
  for(const auto& f : b) {
      State ms = f->mapState(s);
      Action ma = f->mapAction(a);
      sum += (*f)(ms,ma);
  }
  GRBLinExpr rhs = -sum;
#if !NDEBUG
  LOG_DEBUG(lhs << "<=" << rhs);
//GRBConstr constr =
#endif
  addConstraint(lhs, GRB_LESS_EQUAL, rhs);
//#if !NDEBUG
//  LOG_DEBUG("Added constraint:");
//  _lp->update();
//  GRBLinExpr ex = _lp->getRow(constr);
//  LOG_DEBUG(ex);
//#endif
}

void _LP::addAbsVariableBound(double lambda, const std::vector<double>& alpha) {
  // last variable inserted
  const int v_offset = _lp->get(GRB_IntAttr_NumVars);

  // add additional (non-negative) variables to objective for value bound
  for(vector<double>::size_type i = 0; i < alpha.size(); i++) {
    _lp->addVar(0., GRB_INFINITY, 1., GRB_CONTINUOUS, "z"+to_string(i));
  }
  _lp->update();
  LOG_DEBUG("Bounded objective: " << _lp->getObjective());

  // add absolute value constraint
  int offset = v_offset;
  for(vector<double>::size_type i = 0; i < alpha.size(); i++) {
      GRBVar zi = _lp->getVar(offset++);
      addConstraint(zi, GRB_GREATER_EQUAL,  _wvars[i]); // absolute value constraint
      addConstraint(zi, GRB_GREATER_EQUAL, -_wvars[i]);
      addConstraint(zi, GRB_LESS_EQUAL, lambda);
  }
  _lp->update();
}

int _LP::generateLP(const RFunctionVec& C, const RFunctionVec& b, const vector<double>& alpha, const SizeVec& elim_order) {
  // set up LP
  auto tpl = generateObjective("ALP", C, b, alpha);
  std::unordered_map<const _DiscreteFunction<Reward>*, int>& var_offset = std::get<0>(tpl);
  std::vector<DiscreteFunction<Reward>>& empty_fns = std::get<1>(tpl);

  //
  // Run variable elimination to generate further variables/constraints
  //
  try {
    const Size num_factors = _F.getNumFactors();
    LOG_DEBUG("Number of unique factors (S,A) to eliminate: " << num_factors);
    if(elim_order.size() != num_factors) {
      LOG_WARN("Elimination order set has different size from available factors to eliminate");
    }

    int var = _lp->get(GRB_IntAttr_NumVars); // current variable offset
    using range = decltype(_F)::range;
    for(Size v : elim_order) {
        assert(v < _domain->getNumStateFactors() + _domain->getNumActionFactors());
        LOG_DEBUG("Eliminating variable " << v);
        // eliminate variable `v' (either state or action)
        range r = _F.getFactor(v);
        if(!r.hasNext()) {
            LOG_DEBUG("Variable " << v << " not eliminated. It does not exist in FunctionSet.");
            continue;
        }
        EmptyFunction<Reward> E = boost::make_shared<_EmptyFunction<Reward>>(r); // construct new function merely for domain computations
#if !NDEBUG
        const auto& p =
#endif
        var_offset.insert({E.get(), var}); // variable offset in LP
        assert(p.second);
        LOG_DEBUG("Storing offset: " << var << " for replacement function");
        E->computeSubdomain();
        Domain prev_dom_E = E->getSubdomain(); // which still includes `v'
        const subdom_map s_dom(E->getStateFactors());
        const subdom_map a_dom(E->getActionFactors());
        E->eraseFactor(v);
        E->computeSubdomain(); // does not include `v' anymore
#if !NDEBUG
        std::stringstream ss;
        ss << "Newly constructed function E has factor scope: S{ ";
        for(auto s : E->getStateFactors()) {
            ss << s << ", ";
        }
        ss << "} A{ ";
        for(auto a : E->getActionFactors()) {
            ss << a << ", ";
        }
        ss << "};";
        LOG_DEBUG(ss.str());
#endif
        if(E->getSubdomain()->getNumStateFactors() == 0 && E->getSubdomain()->getNumActionFactors() == 0) {
            LOG_DEBUG("Found function with empty scope");
            empty_fns.push_back(E);
        }

        // generate constraints
        _StateActionIncrementIterator saitr(prev_dom_E);
        _lp->addVars(E->getSubdomain()->size()); // note: variable still in [0,inf)
        _lp->update();
        while(saitr.hasNext()) {
            const std::tuple<State,Action>& z = saitr.next();
            const State& s = std::get<0>(z);
            const Action& a = std::get<1>(z);
            // reduced state in E
            State ms = E->mapState(s,s_dom);
            Action ma = E->mapAction(a,a_dom);
            // translate this reduced (ms,ma) pair into an LP variable offset (laid out as <s0,a0>,<s0,a1>,...,<s1,a0>,...)
            const int offset_E = var + ms.getIndex() * E->getSubdomain()->getNumActions() + ma.getIndex();
            LOG_DEBUG("Current offset: " << offset_E);
            // build constraint
            GRBVar v = _lp->getVar(offset_E);
            v.set(GRB_DoubleAttr_LB, -GRB_INFINITY);

            GRBLinExpr lhs = 0;
            r.reset();
            while(r.hasNext()) { // over all functions
                const DiscreteFunction<Reward>& f = r.next();
                // obtain their offset into variable list
                const int offset = var_offset.at(f.get());
                // obtain reduced state
                ms = f->mapState(s,s_dom);
                ma = f->mapAction(a,a_dom);
                // translate this reduced (ms,ma) pair into LP variable offset
                const int f_offset = offset + ms.getIndex() * f->getSubdomain()->getNumActions() + ma.getIndex();
                GRBVar fv = _lp->getVar(f_offset);
                lhs += fv;
            }
            // add constraint (including new variables) to LP
            /*GRBConstr constr =*/ addConstraint(lhs,GRB_LESS_EQUAL,v);

//#if !NDEBUG
//            LOG_DEBUG("Added constraint:");
//            _lp->update();
//            GRBLinExpr ex = _lp->getRow(constr);
//            LOG_DEBUG(ex);
//#endif

        }
        LOG_DEBUG("Function set F before erase of factor: " << v);
        LOG_DEBUG(_F);

        // Erase from var_offset hash table (required to avoid collisions)
        r.reset();
        while(r.hasNext()) {
#if !NDEBUG
            std::size_t k =
#endif
            var_offset.erase(r.next().get());
            assert(k == 1); // exactly one function was removed
        }
        _F.eraseFactor(v);
        _F.insert(E); // store new function (if not empty scope)
        // update variable counter
        var+=E->getSubdomain()->size();

        LOG_DEBUG("Function set F after erase and insertion of new function E: ");
        LOG_DEBUG(_F);
    }

    LOG_DEBUG("Number of unique factors (S,A) after elimination: " << _F.getNumFactors() << " and size: " << _F.size());

    // F does not store empty scope functions
    if(!_F.empty()) {
        return 1; // functions remain in function set: variable elimination failed
    }

    // add remaining constraints (last row)
    // \f$\phi \geq \ldots\f$
    GRBLinExpr lhs = 0;
    for(const auto& e : empty_fns) {
        const int offset = var_offset.at(e.get());
        lhs += _lp->getVar(offset);
    }
    addConstraint(lhs,GRB_LESS_EQUAL,0.);
    _lp->update();

    //write LP to stdout
    LOG_INFO("Generated LP with " << _lp->get(GRB_IntAttr_NumVars) << " variables and " << _lp->get(GRB_IntAttr_NumConstrs) << " constraints.");
    return 0;
  }
  catch(const GRBException& e) {
    cerr << e.getErrorCode() << ": " << e.getMessage() << endl;
  }
  catch(...) {
    cerr << "Unknown exception thrown" << endl;
  }

  return 2;
}

int _LP::generateLiftedLP(const RFunctionVec& C, const RFunctionVec& b, const vector<double>& alpha, const SizeVec& elim_order) {
  // set up LP and abstract away functions C,b
  auto tpl = generateObjective("LiftedALP", C, b, alpha);
  std::unordered_map<const _DiscreteFunction<Reward>*, int>& var_offset = std::get<0>(tpl);
  std::vector<DiscreteFunction<Reward>>& empty_fns = std::get<1>(tpl);

  //
  // Run variable elimination to generate further variables/constraints
  //
  try {
    const Size num_factors = _F.getNumFactors();
    LOG_DEBUG("Number of unique factors (S,A) to eliminate: " << num_factors);
    if(elim_order.size() != num_factors) {
      LOG_WARN("Elimination order set has different size from available factors to eliminate");
    }

    // Using greedy heuristic for elimination order
    std::list<Size> mutable_elim;
    std::copy(elim_order.begin(), elim_order.end(), std::back_inserter(mutable_elim));

    int var = _lp->get(GRB_IntAttr_NumVars); // current variable offset
    using range = decltype(_F)::range;
    while(!mutable_elim.empty()) {
        // determine next best variable to delete
        auto bestIt = elimHeuristic(mutable_elim);
        assert(bestIt != mutable_elim.end());
        Size v = *bestIt;
        mutable_elim.erase(bestIt);
        assert(v < _domain->getNumStateFactors() + _domain->getNumActionFactors());
        LOG_DEBUG("Eliminating variable " << v << " (" << mutable_elim.size() << " to go)");

        // eliminate variable `v' (either state, action, or lifted factor)
        range r = _F.getFactor(v);
        if(!r.hasNext()) {
            LOG_DEBUG("Variable " << v << " not eliminated. It does not exist in FunctionSet.");
            continue;
        }
        EmptyFunction<Reward> E = boost::make_shared<_EmptyFunction<Reward>>(r); // construct new function merely for domain computations
#if !NDEBUG
        const auto& p =
#endif
        var_offset.insert({E.get(), var}); // variable offset in LP
        assert(p.second);
        LOG_DEBUG("Storing offset: " << var << " for replacement function");
        E->computeSubdomain();
        const Domain prev_dom_E = E->getSubdomain(); // which still includes `v' in various factors
        State s_o(prev_dom_E);
        Action a_o(prev_dom_E);
        RLType* prl = nullptr;
        subdom_map s_dom(E->getStateFactors());
        // add lifted factors to original domain mapping
        for(const auto& lf : E->getLiftedFactors()) {
          s_dom.append(lf->getHash());
        }
        const subdom_map a_dom(E->getActionFactors());
        // check if `v' is proper variable in original domain
        bool proper_var = false;
        FactorRange elim_range;
        Size v_loc; // location of `v' in prev_dom_E
        const auto& v_map = v < _domain->getNumStateFactors() ? s_dom.map() : a_dom.map();
        const auto it = v < _domain->getNumStateFactors() ? v_map.find(v) : v_map.find(v-_domain->getNumStateFactors());
        if(it != v_map.end()) {
          proper_var = true;
          v_loc = it->second;
          if(v < _domain->getNumStateFactors()) {
            prl = &s_o;
            elim_range = _domain->getStateRanges()[v];
          }
          else {
            prl = &a_o;
            elim_range = _domain->getActionRanges()[v-_domain->getNumStateFactors()];
          }
          LOG_DEBUG("Eliminating `proper' variable " << v << " with range [" << elim_range.getMin() << "," << elim_range.getMax() << "]");
        }
        else {
          prl = &s_o;
          elim_range = FactorRange(0,1);
          LOG_DEBUG("Eliminating lifted `counter' variable " << v);
        }

        // compress function E
        std::unordered_multimap<std::size_t,Size> delVars;
        auto liftVec = E->compress(&delVars);

        // erase either state or action factor
        E->eraseFactor(v);
        // erase variable from lifted factors
        vector<pair<std::size_t,std::size_t>> liftVec2;
        if(v < _domain->getNumStateFactors()) {
          liftVec2 = E->eraseLiftedFactor(v);
          // merge liftVec with liftVec2
          // TODO: test
          for(const auto& p2 : liftVec2) {
              auto it = std::find_if(liftVec.begin(), liftVec.end(),
                                     [&p2](const pair<std::size_t,std::size_t>& p) { return p2.first == p.second; });
              if(it != liftVec.end()) {
                  LOG_DEBUG("Deleted from count function that was previously compressed.");
                  it->second = p2.second;
                  delVars.emplace(it->first,v);
              }
              else {
                  liftVec.emplace_back(p2.first,p2.second);
                  delVars.emplace(p2.first, v);
              }
          }
        }
        E->computeSubdomain(); // does not include `v' anymore and is compressed
        // local maps
        subdom_map sl_dom(E->getStateFactors());
        for(const auto& lf : E->getLiftedFactors()) {
          sl_dom.append(lf->getHash());
        }
        const subdom_map al_dom(E->getActionFactors());

#if !NDEBUG
        std::ostringstream ss;
        ss << "Newly constructed function E has factor scope: ";
        ss << *E;
        LOG_DEBUG(ss.str());
#endif
        if(E->getSubdomain()->getNumStateFactors() == 0 && E->getSubdomain()->getNumActionFactors() == 0) {
            LOG_DEBUG("Found function with empty scope");
            empty_fns.push_back(E);
        }

        // generate variables and constraints
        _StateActionIncrementIterator saitr(E->getSubdomain());
        _lp->addVars(E->getSubdomain()->size()); // note: variable still in [0,inf)
        _lp->update();
        while(saitr.hasNext()) {
            const std::tuple<State,Action>& z = saitr.next();
            const State& s = std::get<0>(z);
            const Action& a = std::get<1>(z);

            // translate this (s,a) pair into an LP variable offset (laid out as <s0,a0>,<s0,a1>,...,<s1,a0>,...)
            const int offset_E = var + s.getIndex() * E->getSubdomain()->getNumActions() + a.getIndex();
            //LOG_DEBUG("Current offset: " << offset_E);
            // build constraint
            GRBVar lpvar = _lp->getVar(offset_E);
            lpvar.set(GRB_DoubleAttr_LB, -GRB_INFINITY);

            // reconstruct `equivalent' state / action in old domain (all but eliminated var/mod counters)
            if(v < _domain->getNumStateFactors()) {
                a_o = a;
                buildRLType(s, sl_dom, s_o, s_dom);
            }
            else {
                s_o = s;
                buildRLType(a, al_dom, a_o, a_dom);
            }

            // iterate over possible s_o (or a_o) values
            for(Factor fa = elim_range.getMin(); fa <= elim_range.getMax(); fa++) {
                if(proper_var) {
                  prl->setFactor(v_loc, fa);
                }
                // update counters which either include the variable or have been otherwise modified (compression)
                for(const auto& p : liftVec) {
                    const auto o_hash = p.first;
                    const auto n_hash = p.second;
                    Factor val = 0;
                    if(n_hash != _LiftedFactor::EMPTY_HASH) {
                      val = s.getFactor(sl_dom(n_hash));
                    }
                    Factor enabledCt = enabledCount(s, sl_dom, v, fa, delVars.equal_range(o_hash));
                    s_o.setFactor(s_dom(o_hash), val+enabledCt);
                }

                GRBLinExpr lhs = 0;
                r.reset();
                while(r.hasNext()) { // over all functions
                    const DiscreteFunction<Reward>& f = r.next();
                    // obtain their offset into variable list
                    const int offset = var_offset.at(f.get());
                    // obtain reduced state
                    State ms = f->mapState(s_o,s_dom);
                    Action ma = f->mapAction(a_o,a_dom);
                    // translate this reduced (ms,ma) pair into LP variable offset
                    const int f_offset = offset + ms.getIndex() * f->getSubdomain()->getNumActions() + ma.getIndex();
                    GRBVar fv = _lp->getVar(f_offset);
                    lhs += fv;
                }
                // add constraint (including new variables) to LP
                /*GRBConstr constr =*/ addConstraint(lhs,GRB_LESS_EQUAL,lpvar);

//#if !NDEBUG
//                LOG_DEBUG("Added constraint:");
//                _lp->update();
//                GRBLinExpr ex = _lp->getRow(constr);
//                LOG_DEBUG(ex);
//#endif

            }
        }
        LOG_DEBUG("Function set F before erase of factor: " << v);
        LOG_DEBUG(_F);

        // Erase from var_offset hash table (required to avoid collisions)
        r.reset();
        while(r.hasNext()) {
#if !NDEBUG
            std::size_t k =
#endif
            var_offset.erase(r.next().get());
            assert(k == 1); // exactly one function was removed
        }
        _F.eraseFactor(v);
        _F.insert(E); // store new function (if not empty scope)
        // update variable counter
        var+=E->getSubdomain()->size();

        LOG_DEBUG("Function set F after erase and insertion of new function E: ");
        LOG_DEBUG(_F);
    }

    LOG_DEBUG("Number of unique factors (S,A) after elimination: " << _F.getNumFactors() << " and size: " << _F.size());

    // F does not store empty scope functions
    if(!_F.empty()) {
        return 1; // functions remain in function set: variable elimination failed
    }

    // add remaining constraints (last row)
    // \f$\phi \geq \ldots\f$
    GRBLinExpr lhs = 0;
    for(const auto& e : empty_fns) {
        const int offset = var_offset.at(e.get());
        lhs += _lp->getVar(offset);
    }
    addConstraint(lhs,GRB_LESS_EQUAL,0.);
    _lp->update();

    //write LP to stdout
    LOG_INFO("Generated LP with " << _lp->get(GRB_IntAttr_NumVars) << " variables and " << _lp->get(GRB_IntAttr_NumConstrs) << " constraints.");
    _lp->write("lifted.lp");
    return 0;
  }
  catch(const GRBException& e) {
    cerr << e.getErrorCode() << ": " << e.getMessage() << endl;
  }
  catch(...) {
    cerr << "Unknown exception thrown" << endl;
  }

  return 2;
}

int _LP::generateBLP(const RFunctionVec& C, const RFunctionVec& b, const vector<double>& alpha) {
  assert(C.size() == alpha.size());

  try {
    // create the lp
    generateObjective("BLP", alpha);

    //
    // Brute force constraint generation
    //
    _StateActionIncrementIterator saitr(_domain);
    while(saitr.hasNext()) {
        const std::tuple<State,Action>& sa = saitr.next();
        const State& s = std::get<0>(sa);
        const Action& a = std::get<1>(sa);
        // add constraint for those
        addStateActionConstraint(s, a, C, b);
    }
    _lp->update();

    //write LP to stdout
    LOG_INFO("Generated LP with " << _lp->get(GRB_IntAttr_NumVars) << " variables and " << _lp->get(GRB_IntAttr_NumConstrs) << " constraints.");

  } catch(const GRBException& e) {
    LOG_ERROR(e.getErrorCode() << ": " << e.getMessage());
  } catch(...) {
    LOG_ERROR("Unknown exception thrown");
  }

  return 0;
}
#if 0
int _LP::generateVLP(const RFunctionVec& C, const RFunctionVec& b, const vector<double>& alpha, const _DBN& dbn, double gamma) {
  assert(C.size() == alpha.size());

  try {
    // create the lp
    generateObjective("VLP", alpha);

    //
    // Brute force constraint generation
    //
    _StateActionIncrementIterator saitr(_domain);
    vector<double> coeff; // coefficient cache
    coeff.reserve(_wvars.size());
    while(saitr.hasNext()) {
        const std::tuple<State,Action>& sa = saitr.next();
        const State& s = std::get<0>(sa);
        const Action& a = std::get<1>(sa);
        // write out one constraint for those
        GRBLinExpr lhs = 0;
        vector<double>::size_type i = 0;
        for(const auto& f : C) {
            State ms = f->mapState(s);
            coeff[i++] = (*f)(ms);
        }
        lhs.addTerms(coeff.data(), _wvars.data(), _wvars.size());
        // rhs
        double sum = 0.;
        for(const auto& f : b) {
            State ms = f->mapState(s);
            Action ma = f->mapAction(a);
            sum += (*f)(ms,ma);
        }
        GRBLinExpr rhs = sum;
        _StateIncrementIterator sitr(_domain);
        // over all successor states
        while(sitr.hasNext()) {
            const State& n = sitr.next();
            const double prob = gamma * dbn.T(s,a,n);
            i = 0;
            for(const auto& f : C) {
                State ms = f->mapState(n);
                coeff[i++] = prob * (*f)(ms);
            }
            rhs.addTerms(coeff.data(), _wvars.data(), _wvars.size());
        }
//        LOG_DEBUG(lhs << "<=" << rhs);
        addConstraint(lhs, GRB_GREATER_EQUAL, rhs);
    }
    _lp->update();

    //write LP to stdout
    LOG_INFO("Generated LP with " << _lp->get(GRB_IntAttr_NumVars) << " variables and " << _lp->get(GRB_IntAttr_NumConstrs) << " constraints.");

  } catch(const GRBException& e) {
    LOG_ERROR(e.getErrorCode() << ": " << e.getMessage());
  } catch(...) {
    LOG_ERROR("Unknown exception thrown");
  }

  return 0;
}
#endif
int _LP::generateSCALP(const RFunctionVec& C, const RFunctionVec& b, const vector<double>& alpha, const SizeVec& elim_order, double lambda, double beta, double ret_bound) {
  // generate ALP without regularization
  int res = generateLP(C, b, alpha, elim_order);

  if(res) {
      return res; // error
  }

  try {
    _lp->set(GRB_StringAttr_ModelName, "SC-ALP");
    const int v_offset = _lp->get(GRB_IntAttr_NumVars);

    // add additional (non-negative) variables to objective for L_1 regularization
    for(vector<double>::size_type i = 0; i < alpha.size(); i++) {
      _lp->addVar(0., GRB_INFINITY, lambda, GRB_CONTINUOUS, "z"+to_string(i));
//    _lp->addVar(0., GRB_INFINITY, 1., GRB_CONTINUOUS, "z"+to_string(i)); // alternative implementation
    }
#if 0
    // add additional integer variables to objective for basis function scope regularization
    // Note: here, proportional regularization to C state factor scope size
    for(vector<double>::size_type i = 0; i < alpha.size(); i++) {
      const double reg = beta * C[i]->getSubdomain()->getNumStateFactors();
      _lp->addVar(0., 1., reg, GRB_BINARY, "k"+to_string(i));
    }
#endif
    _lp->update();
    LOG_INFO("Objective: " << _lp->getObjective());

    // add absolute value and integer constraints
    int offset = v_offset;
//  const int varno = alpha.size();
    for(vector<double>::size_type i = 0; i < alpha.size(); i++) {
//      GRBVar ki = _lp->getVar(offset+varno); // integer variable
        GRBVar zi = _lp->getVar(offset++);
        addConstraint(zi, GRB_GREATER_EQUAL,  _wvars[i]); // absolute value constraint
        addConstraint(zi, GRB_GREATER_EQUAL, -_wvars[i]);
//      addConstraint(ki, GRB_GREATER_EQUAL, zi/ret_bound); // integer constraint
//      addConstraint(zi, GRB_LESS_EQUAL, lambda); // alternative implementation
    }

    _lp->update();
    //write LP to stdout
    LOG_INFO("Generated LP with " << _lp->get(GRB_IntAttr_NumVars) << " variables and " << _lp->get(GRB_IntAttr_NumConstrs) << " constraints.");
    return 0; // success

  } catch(const GRBException& e) {
    LOG_ERROR(e.getErrorCode() << ": " << e.getMessage());
  } catch(...) {
    LOG_ERROR("Unknown exception thrown");
  }

  return 2;
}

int _LP::solve(crl::_FactoredValueFunction* vfn) {
  try {
    // disable logging
    //_lp->getEnv().set(GRB_IntParam_OutputFlag, 0);

    // run optimization
    _lp->optimize();

    int status = _lp->get(GRB_IntAttr_Status);
    if(status == GRB_OPTIMAL) {
        // query names and values
#if !NDEBUG
        for(const auto& w : _wvars) {
            LOG_DEBUG(w.get(GRB_StringAttr_VarName) << " " << w.get(GRB_DoubleAttr_X));
        }
#endif
        LOG_INFO("The optimal objective of model `" << _lp->get(GRB_StringAttr_ModelName) << "' is "
             << _lp->get(GRB_DoubleAttr_ObjVal));

        // update w vector in value function
        vector<double>& weights = vfn->getWeight();
        assert(_lp->get(GRB_IntAttr_NumVars) >= weights.size());
        const double* xvals = _lp->get(GRB_DoubleAttr_X, _wvars.data(), _wvars.size());
        std::copy(xvals, xvals+_wvars.size(), weights.begin());
        // free Gurobi allocated memory
        delete [] xvals;

        return 0; // success
    }
    else if(status == GRB_UNBOUNDED) {
        LOG_ERROR("The model is unbounded");
        return 1;
    }
    else if(status == GRB_INFEASIBLE) {
        LOG_ERROR("The model is infeasible");
        return 2;
    }
    else {
        LOG_ERROR("Optimization was stopped with status " << status);
        return 3;
    }
  } catch(const GRBException& e) {
    LOG_ERROR(e.getErrorCode() << ": " << e.getMessage());
  } catch(...) {
    LOG_ERROR("Unknown exception thrown");
  }

  return 4;
}

namespace testing {

// Source: http://lpsolve.sourceforge.net/5.5/formulate.htm
//  max(143x + 60y)
// s.t.
//  120x + 210y <= 15000
//  110x + 30y <= 4000
//  x + y <= 75
//  x >= 0, y >= 0
int lp_demo() {
  try {
    GRBEnv env;
    GRBModel model(env);

    model.set(GRB_StringAttr_ModelName, "lp_demo");

    // Create variables
    GRBVar x = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x");
    GRBVar y = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y");
    model.update();

    // objective
    model.setObjective(143 * x + 60 * y, GRB_MAXIMIZE);

    // constraint
    model.addConstr(120 * x + 210 * y <= 15000, "c0");
    model.addConstr(110 * x + 30 *  y <=  4000, "c1");
    model.addConstr(      x +       y <=    75, "c2");

    // run optimization
    model.optimize();

    // print results
    int status = model.get(GRB_IntAttr_Status);
    if(status == GRB_OPTIMAL) {
      cout << x.get(GRB_StringAttr_VarName) << " " << x.get(GRB_DoubleAttr_X) << endl;
      cout << y.get(GRB_StringAttr_VarName) << " " << y.get(GRB_DoubleAttr_X) << endl;
      cout << "The optimal objective of model `" << model.get(GRB_StringAttr_ModelName) << "' is "
           << model.get(GRB_DoubleAttr_ObjVal) << endl;
      return 0;
    }
    else if(status == GRB_UNBOUNDED) {
      cerr << "The model is unbounded" << endl;
      return 1;
    }
    else if ((status != GRB_INF_OR_UNBD) && (status != GRB_INFEASIBLE)) {
      cerr << "Optimization was stopped with status " << status << endl;
      return 2;
    }
    else {
      cerr << "The model is infeasible" << endl;
      return 3;
    }
  } catch(const GRBException& e) {
    cerr << e.getErrorCode() << ": " << e.getMessage() << endl;
  } catch(...) {
    cerr << "Unknown exception thrown" << endl;
  }

  return 4;
}

} // namespace testing

} // namespace gurobi
