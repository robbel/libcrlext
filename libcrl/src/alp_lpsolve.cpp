/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include "crl/alp_lpsolve.hpp"
#include <unordered_map>

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
// Regarding LP variable ordering:
// - First go the w_i variables, then the `scope variables' associated with each function in C, then those with b, then those from variable elimination.
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
  for(vector<double>::size_type w = 1; w <= alpha.size(); w++) {
    set_col_name(_lp, w, &string("w"+to_string(w))[0]); // TODO: optional if performance becomes an issue
  }

  // make building the model faster if it is done row by row
  set_add_rowmode(_lp, TRUE);

  //
  // Objective function (minimization is lpsolve default)
  //

  {
    vector<int> colno = cpputil::ordered_vec(alpha.size(), 1); // 1-offset column numbering
    vector<REAL>& row = const_cast<vector<REAL>&>(alpha); // API interface requires non-const pointer
    if(!set_obj_fnex(_lp, row.size(), row.data(), colno.data())) {
        return 2; // objective function setting failed
    }
  }

  //
  // generate equality constraints to abstract away basis (C) functions
  //

  // A mapping from function to lp-variable offset (used during variable elimination)
  std::unordered_map<const _DiscreteFunction<Reward>*, int> var_offset;
  int var = alpha.size()+1; // the offset at which to insert more variables
  int wi = 1; // corresponding to active w_i variable
  int colno[2]; // constraints take a particularly simple form and involve two variables each
  REAL row[2];
  for(const auto& f : C) {
      const _FDiscreteFunction<Reward>* pf = static_cast<_FDiscreteFunction<Reward>*>(f.get());
      var_offset.insert({pf,var});
      const vector<Reward>& vals = pf->values(); // optimization: direct access of all values in subdomain
      for(auto v : vals) {
          // add lpsolve constraint corresponding to w_i
          colno[0] = wi;
          row[0] = v;
          colno[1] = var++;
          row[1] = -1.;
          if(!add_constraintex(_lp, 2, row, colno, EQ, 0.)) {
              return 3; // adding of constraint failed
          }
      }
      F.insert(std::move(f)); // store function in set for variable elimination
      wi++;
  }

  //
  // generate equality constraints to abstract away target (b) functions
  //

  for(const auto& f : b) {
      const _FDiscreteFunction<Reward>* pf = static_cast<_FDiscreteFunction<Reward>*>(f.get());
      var_offset.insert({pf,var});
      const vector<Reward>& vals = pf->values();
      for(auto v : vals) {
          // add lpsolve equality constraint
          colno[0] = var++;
          row[0] = 1.;
          if(!add_constraintex(_lp, 1, row, colno, EQ, v)) {
              return 4; // adding of constraint failed
          }
      }
      F.insert(std::move(f));
  }

  // sanity check
  if(get_Ncolumns(_lp) != lp_vars) {
      return 5; // not all lp variables have been added
  }
  if(var != lp_vars+1) {
      return 6; // counting is incorrect
  }

  //
  // Run variable elimination to generate further variables/constraints
  //

  // TODO can I resize lp here (depending on observed performance)

  const Size num_states = _domain->getNumStateFactors();
  const Size num_factors = F.getNumFactors();
  std::cout << "[DEBUG]: Number of unique factors (S,A) to eliminate: " << num_factors << std::endl;
  if(elim_order.size() != num_factors) {
    std::cout << "[WARNING]: Elimination order set has different size from available factors to eliminate" << std::endl;
  }

  std::vector<DiscreteFunction<Reward>> empty_fns; // store for functions that have reached empty scope (for last constraint)
  using range = decltype(F)::range;
  for(Size v : elim_order) {
      assert(v < num_states + _domain->getNumActionFactors());
      std::cout << "[DEBUG]: Eliminating variable " << v << std::endl;

      // 1. collect all functions that have dependence on v: E
      //      also obtain their offset into variable list
      // 2. create new function e with joint scope sc[E] \ {v}
      // 3. add constraints:
      //      for each dom[e], introduce variable u_z^e:
      //          for each v add a constraint:
      //          u_z^e >= \sum affected function instantiation variables..
      //    store offset to beginning of those variables, as usual
      //    store new function in FunctionSet

      // eliminate variable `v' (either state or action)
      range r = F.getFactor(v);
      EmptyFunction<Reward> E = boost::make_shared<_EmptyFunction<Reward>>(r); // construct new function merely for domain computations
      var_offset.insert({E.get(),var}); // variable offset in LP
      E->computeSubdomain();
      Domain prev_dom_E = E->getSubdomain(); // which still includes `v'
      const subdom_map s_dom(E->getStateFactors());
      const subdom_map a_dom(E->getActionFactors());
      if(v < num_states) {
        E->eraseStateFactor(v);
      }
      else {
        E->eraseActionFactor(v-num_states);
      }
      E->computeSubdomain(); // does not include `v' anymore

      std::cout << "[DEBUG]: Newly constructed function E has factor scope: S{ ";
      for(auto s : E->getStateFactors()) {
          std::cout << s << ", ";
      }
      std::cout << "} A{ ";
      for(auto a : E->getActionFactors()) {
          std::cout << a << ", ";
      }
      std::cout << "};" << std::endl;

      if(!E->getSubdomain()->getNumStateFactors() && !E->getSubdomain()->getNumStateFactors()) {
          std::cout << "[DEBUG]: Found function with empty scope" << std::endl;
          empty_fns.push_back(E);
      }

      // generate constraints
      _StateActionIncrementIterator saitr(prev_dom_E);
      while(saitr.hasNext()) {
          const std::tuple<State,Action>& z = saitr.next();
          const State& s = std::get<0>(z);
          const Action& a = std::get<1>(z);
          // reduced state in E
          State ms = E->mapState(s,s_dom);
          Action ma = E->mapAction(a,a_dom);
          // translate this reduced (ms,ma) pair into an LP variable offset (laid out as <s0,a0>,<s0,a1>,...,<s1,a0>,...)
          const int offset_E = var + ms.getIndex() * E->getSubdomain()->getNumActions() + ma.getIndex();
          // build constraint
          vector<REAL> row; // TODO: move out of loop
          vector<int> colno; // 1-offset column numbering
          colno.push_back(offset_E);
          row.push_back(-1.);

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
              // add to constraint
              colno.push_back(f_offset);
              row.push_back(1.);
          }
          // add constraint (including new variables) to LP
          if(!add_constraintex(_lp, row.size(), row.data(), colno.data(), LE, 0.)) {
              return 7; // adding of constraint failed
          }
      }

      std::cout << "[DEBUG]: Function set F before erase of factor: " << v << std::endl;
      std::cout << F << std::endl;

      F.eraseFactor(v);
      // could erase from var_offset hash table too
      F.insert(E); // store new function (if not empty scope)
      // update variable counter
      var = get_Ncolumns(_lp) + 1;

      std::cout << "[DEBUG] Function set F after erase and insertion of new function E: " << std::endl;
      std::cout << F << std::endl;
  }

  std::cout << "[DEBUG]: Number of unique factors (S,A) after elimination: " << F.getNumFactors() << " and size: " << F.size() << std::endl;

  // F does not store empty scope functions
  if(!F.empty()) {
      return 8; // functions remain in function set: variable elimination failed
  }

  // TODO verify var not off by 1
  // add remaining constraints (last row)
  // \f$\phi \geq \ldots\f$
  vector<REAL> lrow;
  vector<int> lcolno; // 1-offset column numbering
  for(const auto& e : empty_fns) {
      const int offset = var_offset.at(e.get());
      lcolno.push_back(offset);
      lrow.push_back(1);
  }
  if(!add_constraintex(_lp, lrow.size(), lrow.data(), lcolno.data(), LE, 0.)) {
      return 9; // adding of constraint failed
  }

  // add one final constraint (not required for ALP as phi is no extra variable)
  // \f$\phi=0\f$

  // debug out
  set_add_rowmode(_lp, FALSE);
  write_LP(_lp, stdout);

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
