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
#include "crl/function.hpp"
#include <boost/shared_ptr.hpp>

//
// LPSolve5.5 specific code to support the \a ALPPlanner
//

// Some notes on lpsolve (from the FAQ):
//   If the model is build column by column, then it is strongly suggested to use add_columnex instead of add_column because
//   add_columnex gives the possibility to only supply the non-zero elements and that speeds up building the model considerably,
//   especially if the matrix is sparse (a lot of zero elements).
// Version 5 has a new API call set_add_rowmode that makes add_constraint, str_add_constraint spectacular faster
//   If the model is build row by row, then it is strongly suggested to use add_constraintex instead of add_constraint because add_constraintex
//   gives the possibility to only supply the non-zero elements and that speeds up building the model considerably, especially if the matrix
//   is sparse (a lot of zero elements).
// Also see http://cgi.csc.liv.ac.uk/~anshul/BIMatrix/lpsolve/leader.c for another example

namespace crl {

/// \brief typedef for vectors of flat functions
template<class T>
using FunctionVec = std::vector<DiscreteFunction<T>>;
/// \brief typedef for vectors of (flat) reward functions
typedef FunctionVec<Reward> RFunctionVec;

} // namespace crl

namespace lpsolve {

/**
 * \brief A class that abstracts an lpsolve LP
 * All specific LP solver dependencies should be restricted to this hpp/cpp file
 */
class _LP {
private:
  lprec *_lp;
public:
  /// \brief ctor
  _LP()
  : _lp(nullptr) {}
  /// \brief dtor
  ~_LP();

  /// \brief Given targets \f$C\f$ and \f$\mathbf{b}\f$ compute polynomial set of constraints
  /// \return 0 iff successful
  int generateLP(const crl::RFunctionVec& C, const crl::RFunctionVec& b, const crl::SizeVec& elim_order);
  /// \brief Solve this LP
  /// \return 0 iff successful
  int solve();
};
typedef boost::shared_ptr<_LP> LP;

namespace testing {

/// \brief A basic LP example that ships with lpsolve
/// Source: http://lpsolve.sourceforge.net/5.5/formulate.htm
int lp_demo();
/// \brief Some random experiments with lpsolve
void lp_exp();

}

} // namespace lpsolve

#endif /*ALP_LPSOLVE_HPP_*/
