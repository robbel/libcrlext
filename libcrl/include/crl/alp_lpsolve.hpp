/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef ALP_LPSOLVE_HPP_
#define ALP_LPSOLVE_HPP_

#include <iostream>
#include <lpsolve/lp_lib.h>
#include "crl/alp.hpp"

//
// LPSolve5.5 specific code to support the \a ALPPlanner
//

// Some notes on lpsolve (from the FAQ):
//   If the model is build column by column, then it is strongly suggested to use add_columnex instead of add_column because
//   add_columnex gives the possibility to only supply the non-zero elements and that speeds up building the model considerably,
//   especially if the matrix is sparse (a lot of zero elements).
// Version 5 has a new API call set_add_rowmode that makes add_constraint, str_add_constraint spectacular faster
// Also see http://cgi.csc.liv.ac.uk/~anshul/BIMatrix/lpsolve/leader.c for another example

namespace crl {

/// \brief typedef for vectors of discrete functions
template<class T>
using FunctionVec = std::vector<DiscreteFunction<T>>;
/// \brief typedef for vectors of Reward functions
typedef FunctionVec<Reward> RFunctionVec;

} // namespace crl

namespace lpsolve {

/**
 * \brief A class that abstracts an lpsolve LP
 * All specific LP solver dependencies should be restricted to this hpp/cpp file
 * TODO: could implement iterative constraint generation if performance becomes prohibitive
 */
class _LP {
private:
  /// \brief Default upper bound on variables in LP
  static const int MAX_COL = 1000;
  /// \brief Chosen upper bound on variables in LP
  const int _colsize;
  /// \brief The domain which includes all state and action factors
  const crl::Domain _domain;
  /// \brief Storage for functions generated during variable elimination
  crl::FunctionSet<crl::Reward> F;
  /// \brief The lpsolve LP
  lprec *_lp;
public:
  /// \brief ctor
  _LP(const crl::Domain& domain, int colsize = MAX_COL)
  : _colsize(colsize), _domain(domain), F(domain), _lp(nullptr) { }
  /// \brief dtor
  ~_LP();

  /// \brief Given targets \f$C\f$ and \f$\mathbf{b}\f$ compute polynomial set of constraints
  /// \return 0 iff successful
  int generateLP(const crl::RFunctionVec& C, const crl::RFunctionVec& b, const std::vector<double>& alpha, const crl::SizeVec& elim_order);
  /// \brief Given targets \f$C\f$ and \f$\mathbf{b}\f$ compute possibly exponential set of constraints
  /// \return 0 iff successful
  int generateBLP(const crl::RFunctionVec& C, const crl::RFunctionVec& b, const std::vector<double>& alpha);
  /// \brief Solve this LP
  /// \return 0 iff successful
  int solve(crl::_FactoredValueFunction* vfn);
};
typedef boost::shared_ptr<_LP> LP;

namespace testing {

/// \brief A basic LP example that ships with lpsolve
/// Source: http://lpsolve.sourceforge.net/5.5/formulate.htm
int lp_demo();

}

} // namespace lpsolve

#endif /*ALP_LPSOLVE_HPP_*/
