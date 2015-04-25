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

  }
  catch(const GRBException& e) { }

  throw cpputil::InvalidException("not implemented yet.");

  return 0;
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
    else if ((status != GRB_INF_OR_UNBD) && (status != GRB_INFEASIBLE)) {
        LOG_ERROR("Optimization was stopped with status " << status);
        return 2;
    }
    else {
        LOG_ERROR("The model is infeasible");
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
