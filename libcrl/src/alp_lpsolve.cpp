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
using namespace std;

namespace lpsolve {

//
// LP abstraction implementation
//

_LP::~_LP() {
  if(_lp) {
      delete_lp(_lp);
  }
}

//
// LP constraint/variable ordering:
// first go w_i (for all funcs in C), then `equality' variables associated with C, then those with b.
//

int _LP::generateLP(const RFunctionVec& C, const RFunctionVec& b, const std::vector<double>& alpha, const SizeVec& elim_order) {
//assert(C.size() == alpha.size());
  F.clear();

  // compute lower bound on lp size (before variable elimination algorithm)
  int lp_vars = alpha.size(); // corresponding to the w_i
  for(const auto& f : C) {
      lp_vars += f->getSubdomain()->size(); // for each instantiation of C's subdomain
  }
  for(const auto& f : b) {
      lp_vars += f->getSubdomain()->size(); // for each instantiation of b's subdomain
  }

  // create LP with this number of variables (will require resizing later)
  _lp = make_lp(0, lp_vars);
  if(_lp == nullptr) {
    return 1; // couldn't construct a new model
  }

  // note the 1-offset in lpsolve
  for(RFunctionVec::size_type w = 1; w <= alpha.size(); w++) {
    set_col_name(_lp, w, &string("w"+to_string(w))[0]);
  }

  // make building the model faster if it is done row by row
  // TODO: test whether this disallows further column naming or lp column-number increases below!
  set_add_rowmode(_lp, TRUE);

  //
  // Objective function
  //
  {
    vector<REAL>& row = const_cast<vector<REAL>&>(alpha); // API interface requires non-const pointer
    vector<int> colno = cpputil::ordered_vec(alpha.size(), 1); // 1-offset column numbering
    if(!set_obj_fnex(_lp, row.size(), row.data(), colno.data())) {
        return 2; // objective function setting failed
    }
  }

  //
  // generate equality constraints to abstract away basis functions
  //
  int var = alpha.size()+1; // insert more variables
  int fi = 1; // corresponding to active w_i variable
  for(const auto& f : C) {
      const _FDiscreteFunction<Reward>* pf = static_cast<_FDiscreteFunction<Reward>*>(f.get()); // assumption: flat representation
      const vector<Reward>& vals = pf->values(); // optimization: direct access of all values in subdomain
      _StateActionIncrementIterator saitr(f->getSubdomain());
      for(auto v : vals) {
          const std::tuple<State,Action>& z = saitr.next();
          // create new lp variable
          const State& s = std::get<0>(z);
          const Action& a = std::get<1>(z);
          string varname = "f" + to_string(fi) + "@s: " + to_string(s.getIndex()) + ", a: " + to_string(a.getIndex());
          set_col_name(_lp, var++, &varname[0]);
          // add lpsolve constraint corresponding to w_i
          // todo ...
          cout << v << endl;
      }
      F.insert(std::move(f)); // TODO move ok here?
      fi++;
  }

  //
  // generate equality constraints to abstract away target functions
  //
  for(const auto& f : b) {
      const _FDiscreteFunction<Reward>* pf = static_cast<_FDiscreteFunction<Reward>*>(f.get()); // assumption: flat representation
      const vector<Reward>& vals = pf->values(); // optimization: direct access of all values in subdomain
      _StateActionIncrementIterator saitr(f->getSubdomain());
      for(auto v : vals) {
          const std::tuple<State,Action>& z = saitr.next();
          // create new lp variable
          const State& s = std::get<0>(z);
          const Action& a = std::get<1>(z);
          string varname = "b" + to_string(fi) + "@s: " + to_string(s.getIndex()) + ", a: " + to_string(a.getIndex());
          set_col_name(_lp, var++, &varname[0]);
          // add lpsolve equality constraint
          // todo ...
          cout << v << endl;
      }
      F.insert(f);
      fi++;
  }

  //
  // Now F contains all the functions involved in the LP. Run variable elimination to generate constraints
  //
  using range = decltype(F)::range;
  const Size num_states = _domain->getNumStateFactors();
  for(Size var : elim_order) {
      // eliminate variable `v'
      if(var < num_states) { // a state factor, as per convention
          // todo
          F.eraseStateFactor(var);
      }
      else { // an action factor
          // todo
          F.eraseActionFactor(var-num_states);
      }
  }

  // debug out
  set_add_rowmode(_lp, FALSE);
  write_LP(_lp, stdout);

  // add remaining constraints
  // \f$\phi \geq \ldots\f$

  // add one final constraint
  // \f$\phi=0\f$

  return 0;
}

int _LP::solve(_FactoredValueFunction* vfn) {
  // solve minimization
  set_verbose(_lp, IMPORTANT);

  // Let lpsolve calculate a solution
  int ret = ::solve(_lp);
  if(ret != OPTIMAL)
    return ret;

  REAL *ptr_var; // memory maintained by lpsolve
  get_ptr_variables(_lp, &ptr_var);

  // update w vector in vfn
  std::vector<double>& weights = vfn->getWeight();
  std::copy(ptr_var, ptr_var+weights.size(), weights.begin());

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

    // equivalent call with lpsolve-maintained memory
    REAL *ptr_var;
    get_ptr_variables(lp, &ptr_var);

    for(j = 0; j < Ncol; j++)
      printf("%s: %f\n", get_col_name(lp, j + 1), ptr_var[j]);
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
