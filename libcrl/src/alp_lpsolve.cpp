/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include "crl/alp_lpsolve.hpp"

using namespace crl;

namespace lpsolve {

//
// LP abstraction implementation
//

_LP::~_LP() {
  if(_lp) {
      delete_lp(_lp);
  }
}

int _LP::generateLP(const RFunctionVec& C, const RFunctionVec& b, const SizeVec& elim_order) {
  F.clear();

  // compute lower bound on lp size
  int lp_vars = 0;
  for(const auto& f : C) {
      lp_vars += f->getSubdomain()->size();
  }
  for(const auto& f : b) {
      lp_vars += f->getSubdomain()->size();
  }

  // create LP with this number of variables (will require resizing later)
  _lp = make_lp(0, lp_vars);
  if(_lp == nullptr) {
    return 1; // couldn't construct a new model
  }

  // generate equality constraints to abstract away basis functions
//  int var = 1; // 1-offset
  for(RFunctionVec::size_type i = 0; i < C.size(); i++) {
      const DiscreteFunction<Reward>& f = C[i];
      _StateActionIncrementIterator saitr(f->getSubdomain());
      while(saitr.hasNext()) {
//          const std::tuple<State,Action>& z = saitr.next();
//          const State& s = std::get<0>(z);
//          const Action& a = std::get<1>(z);
//          // create new lp variable and add constraint
//          set_col_name(_lp, var++, &std::string(f->getName() + s + a)[0]);



      }
      F.insert(f);
  }
  // generate equality constraints to abstract away target functions
  for(const auto& f : b) {
      _StateActionIncrementIterator saitr(f->getSubdomain());
      while(saitr.hasNext()) {
          //const std::tuple<State,Action>& z = saitr.next();
          // create new lp variable and add constraint

      }
      F.insert(f);
  }

  // Now F contains all the functions involved in the LP. Run variable elimination to generate constraints
  using range = decltype(F)::range;
  const Size num_states = _domain->getNumStateFactors();
//  typedef F::range range;
  for(Size v : elim_order) {
      // eliminate variable `v'
      if(v < num_states) { // a state factor, as per convention
          // todo
          F.eraseStateFactor(v);
      }
      else { // an action factor
          // todo
          F.eraseActionFactor(v-num_states);
      }
  }

  // add remaining constraints
  // \f$\phi \geq \ldots\f$

  // add one final constraint
  // \f$\phi=0\f$

  return 0;
}

int _LP::solve(const std::vector<double>& alpha, _FactoredValueFunction* vfn) {
  // solve minimization
  // \f$\sum_i \alpha_i w_i\f$
  // subject to generated constraints

  // update w vector in vfn
  return 0;
}

//
// Various lpsolve testing methods
//

namespace testing {

// Source: http://lpsolve.sourceforge.net/5.5/formulate.htm
//  max(143x + 60y)
// s.t.
//  120x + 210y <= 15000
//  110x + 30y <= 4000
//  x + y <= 75
//  x >= 0, y >= 0
int lp_demo() {
  lprec *lp;
  int Ncol, *colno = nullptr, j, ret = 0;
  REAL *row = nullptr;

  // We will build the model row by row so we start with creating a model with 0 rows and 2 columns
  Ncol = 2; // there are two variables in the model
  lp = make_lp(0, Ncol);
  if(lp == nullptr)
    ret = 1; // couldn't construct a new model...

  if(ret == 0) {
    // let us name our variables. Not required, but can be useful for debugging
    set_col_name(lp, 1, &std::string("x")[0]);
    set_col_name(lp, 2, &std::string("y")[0]);

    // create space large enough for one row
    colno = new int[Ncol];
    row = new REAL[Ncol];
    if((colno == nullptr) || (row == nullptr))
      ret = 2;
  }

  if(ret == 0) {
    set_add_rowmode(lp, TRUE);  // makes building the model faster if it is done rows by row

    // construct first row (120 x + 210 y <= 15000)
    j = 0;

    colno[j] = 1; // first column (index)
    row[j++] = 120; // associated value in row 0

    colno[j] = 2; // second column (index)
    row[j++] = 210; // associated value in row 0

    // add the row to lpsolve
    if(!add_constraintex(lp, j, row, colno, LE, 15000))
      ret = 3;
  }

  if(ret == 0) {
    // construct second row (110 x + 30 y <= 4000)
    j = 0;

    colno[j] = 1;
    row[j++] = 110;

    colno[j] = 2;
    row[j++] = 30;

    if(!add_constraintex(lp, j, row, colno, LE, 4000))
      ret = 3;
  }

  if(ret == 0) {
    // construct third row (x + y <= 75)
    j = 0;

    colno[j] = 1;
    row[j++] = 1;

    colno[j] = 2;
    row[j++] = 1;

    if(!add_constraintex(lp, j, row, colno, LE, 75))
      ret = 3;
  }

  if(ret == 0) {
    set_add_rowmode(lp, FALSE); // rowmode should be turned off again when done building the model

    // set the objective function (143 x + 60 y)
    j = 0;

    colno[j] = 1; // first column
    row[j++] = 143;

    colno[j] = 2; // second column
    row[j++] = 60;

    // set the objective in lpsolve
    if(!set_obj_fnex(lp, j, row, colno))
      ret = 4;
  }

  if(ret == 0) {
    // set the object direction to maximize
    set_maxim(lp);

    // just out of curiosity, now show the model in lp format on screen
    // this only works if this is a console application. If not, use write_lp and a filename
    write_LP(lp, stdout);
    // write_lp(lp, "model.lp");

    // I only want to see important messages on screen while solving
    set_verbose(lp, IMPORTANT);

    // Now let lpsolve calculate a solution
    ret = solve(lp);
    if(ret == OPTIMAL)
      ret = 0;
    else
      ret = 5;
  }

  if(ret == 0) {
    // a solution is calculated, now lets get some results

    // objective value
    printf("Objective value: %f\n", get_objective(lp));

    // variable values
    get_variables(lp, row);
    for(j = 0; j < Ncol; j++)
      printf("%s: %f\n", get_col_name(lp, j + 1), row[j]);
  }

  // free allocated memory
  if(row != nullptr)
    delete [] row;
  if(colno != nullptr)
    delete [] colno;

  if(lp != nullptr) {
    // clean up such that all used memory by lpsolve is freed
    delete_lp(lp);
  }

  return ret;
}

} // namespace testing

} // namespace lpsolve
