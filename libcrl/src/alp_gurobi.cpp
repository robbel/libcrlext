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

namespace gurobi {

int _LP::generateVLP(const RFunctionVec& C, const RFunctionVec& b, const vector<double>& alpha, const _DBN& dbn, double gamma) {
  assert(C.size() == alpha.size());

  _wvars.clear();
  _wvars.reserve(alpha.size());

  try {
    // create the lp
    _env = boost::make_shared<GRBEnv>();
    _lp  = boost::make_shared<GRBModel>(*_env);
    _lp->set(GRB_StringAttr_ModelName, "VLP");

    // add the weight variables with [-inf,inf] bounds along with the objective coefficients
    for(vector<double>::size_type i = 0; i < alpha.size(); i++) {
      GRBVar v = _lp->addVar(-GRB_INFINITY, GRB_INFINITY, alpha[i], GRB_CONTINUOUS, "w"+to_string(i));
      _wvars.push_back(std::move(v));
    }

    // The objective is to minimize the costs
    _lp->set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    _lp->update();
    LOG_INFO("Objective: " << _lp->getObjective());

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
        _lp->addConstr(lhs, GRB_GREATER_EQUAL, rhs);
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

int _LP::generateBLP(const RFunctionVec& C, const RFunctionVec& b, const vector<double>& alpha) {
  assert(C.size() == alpha.size());

  _wvars.clear();
  _wvars.reserve(alpha.size());

  try {
    // create the lp
    _env = boost::make_shared<GRBEnv>();
    _lp  = boost::make_shared<GRBModel>(*_env);
    _lp->set(GRB_StringAttr_ModelName, "BLP");

    // add the weight variables with [-inf,inf] bounds along with the objective coefficients
    for(vector<double>::size_type i = 0; i < alpha.size(); i++) {
      GRBVar v = _lp->addVar(-GRB_INFINITY, GRB_INFINITY, alpha[i], GRB_CONTINUOUS, "w"+to_string(i));
      _wvars.push_back(std::move(v));
    }

    // The objective is to minimize the costs
    _lp->set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    _lp->update();
    LOG_INFO("Objective: " << _lp->getObjective());

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
//        LOG_DEBUG(lhs << "<=" << rhs);
        _lp->addConstr(lhs, GRB_LESS_EQUAL, rhs);
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

int _LP::generateLP(const RFunctionVec& C, const RFunctionVec& b, const vector<double>& alpha, const SizeVec& elim_order) {
  //assert(C.size() == alpha.size());
  F.clear();

  _wvars.clear();
  _wvars.reserve(alpha.size());

  try {
    // create the lp
    _env = boost::make_shared<GRBEnv>();
    _lp  = boost::make_shared<GRBModel>(*_env);
    _lp->set(GRB_StringAttr_ModelName, "ALP");

    // add the weight variables with [-inf,inf] bounds along with the objective coefficients
    for(vector<double>::size_type i = 0; i < alpha.size(); i++) {
      GRBVar v = _lp->addVar(-GRB_INFINITY, GRB_INFINITY, alpha[i], GRB_CONTINUOUS, "w"+to_string(i));
      _wvars.push_back(std::move(v));
    }

    // The objective is to minimize the costs
    _lp->set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    _lp->update();
    LOG_INFO("Objective: " << _lp->getObjective());

    //
    // generate equality constraints to abstract away basis (C) functions
    //

    // A mapping from function to lp-variable offset (used during variable elimination)
    std::unordered_map<const _DiscreteFunction<Reward>*, int> var_offset;
    // store for functions that have reached empty scope (those are used in last constraint)
    std::vector<DiscreteFunction<Reward>> empty_fns;


    int var = alpha.size(); // the offset at which to insert more variables
    int wi = 0; // corresponding to active w_i variable
    for(const auto& f : C) {
        const auto& p = var_offset.insert({f.get(),var});
        if(!p.second) {
          assert(false);
        }
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
            _lp->addConstr(lhs, GRB_EQUAL, v);
            var++;
            cc++;
        }
        // test for constant basis
        if(f->getSubdomain()->getNumStateFactors() == 0 && f->getSubdomain()->getNumActionFactors() == 0) {
            LOG_DEBUG("Found constant basis function with empty scope");
            empty_fns.push_back(std::move(f));
        }
        else {
            F.insert(std::move(f)); // store function in set for variable elimination
        }
        wi++;
    }

    //
    // generate equality constraints to abstract away target (b) functions
    //

    int bb = 0;
    for(const auto& f : b) {
        const auto& p = var_offset.insert({f.get(),var});
        if(!p.second) {
            assert(false);
        }
        int bv = 0;
        _StateActionIncrementIterator saitr(f->getSubdomain());
        while(saitr.hasNext()) {
            const std::tuple<State,Action>& sa = saitr.next();
            const State& s = std::get<0>(sa);
            const Action& a = std::get<1>(sa);
            // add variable
            GRBVar v = _lp->addVar(-GRB_INFINITY, GRB_INFINITY, 0., GRB_CONTINUOUS, "b"+to_string(bb)+"_"+to_string(bv));
            _lp->update();
            _lp->addConstr(v, GRB_EQUAL, (*f)(s,a));
            var++;
            bv++;
        }
        F.insert(std::move(f));
        bb++;
    }

    //
    // Run variable elimination to generate further variables/constraints
    //

    const Size num_factors = F.getNumFactors();
    LOG_DEBUG("Number of unique factors (S,A) to eliminate: " << num_factors);
    if(elim_order.size() != num_factors) {
      LOG_WARN("Elimination order set has different size from available factors to eliminate");
    }

    using range = decltype(F)::range;
    for(Size v : elim_order) {
        assert(v < _domain->getNumStateFactors() + _domain->getNumActionFactors());
        LOG_DEBUG("Eliminating variable " << v);
        // eliminate variable `v' (either state or action)
        range r = F.getFactor(v);
        if(!r.hasNext()) {
            LOG_DEBUG("Variable " << v << " not eliminated. It does not exist in FunctionSet.");
            continue;
        }
        EmptyFunction<Reward> E = boost::make_shared<_EmptyFunction<Reward>>(r); // construct new function merely for domain computations
        const auto& p = var_offset.insert({E.get(), var}); // variable offset in LP
        LOG_DEBUG("Storing offset: " << var << " for replacement function");
        if(!p.second) {
            assert(false);
        }
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
            /*GRBConstr constr =*/ _lp->addConstr(lhs,GRB_LESS_EQUAL,v);
/*
#if !NDEBUG
            LOG_DEBUG("Added constraint:");
            _lp->update();
            GRBLinExpr ex = _lp->getRow(constr);
            LOG_DEBUG(ex);
#endif
*/
        }
        LOG_DEBUG("Function set F before erase of factor: " << v);
        LOG_DEBUG(F);

        // Erase from var_offset hash table (required to avoid collisions)
        r.reset();
        while(r.hasNext()) {
            std::size_t k = var_offset.erase(r.next().get());
            if(k != 1) {
                assert(false);
            }
        }
        F.eraseFactor(v);
        F.insert(E); // store new function (if not empty scope)
        // update variable counter
        var+=E->getSubdomain()->size();

        LOG_DEBUG("Function set F after erase and insertion of new function E: ");
        LOG_DEBUG(F);
    }

    LOG_DEBUG("Number of unique factors (S,A) after elimination: " << F.getNumFactors() << " and size: " << F.size());

    // F does not store empty scope functions
    if(!F.empty()) {
        return 1; // functions remain in function set: variable elimination failed
    }

    // add remaining constraints (last row)
    // \f$\phi \geq \ldots\f$
    GRBLinExpr lhs = 0;
    for(const auto& e : empty_fns) {
        const int offset = var_offset.at(e.get());
        lhs += _lp->getVar(offset);
    }
    _lp->addConstr(lhs,GRB_LESS_EQUAL,0.);
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
