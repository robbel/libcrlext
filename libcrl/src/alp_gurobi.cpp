/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <boost/scoped_ptr.hpp>

#include "crl/alp_gurobi.hpp"


using namespace std;
using namespace crl;

namespace gurobi {

int _LP::generateLP(const RFunctionVec& C, const RFunctionVec& b, const vector<double>& alpha, const SizeVec& elim_order) {
  F.clear();

  // todo

  return 0;

}

int _LP::generateBLP(const RFunctionVec& C, const RFunctionVec& b, const vector<double>& alpha) {
  F.clear();

  // todo

  return 0;

}


int _LP::solve(crl::_FactoredValueFunction* vfn) {

  // todo

  return 0;
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
      cout << "The optimal objective is " << model.get(GRB_DoubleAttr_ObjVal) << endl;
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
  } catch(GRBException e) {
    cerr << e.getErrorCode() << endl;
    cerr << e.getMessage() << endl;
  } catch(...) {
    cerr << "Unknown exception thrown" << endl;
  }

  return 4;
}

} // namespace testing

} // namespace gurobi
