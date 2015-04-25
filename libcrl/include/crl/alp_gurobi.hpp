/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef ALP_GUROBI_HPP_
#define ALP_GUROBI_HPP_

#include <iostream>

#include "gurobi_c++.h"
#include "crl/alp.hpp"

//
// Gurobi specific code to support the \a ALPPlanner
//

namespace crl {

/// \brief typedef for vectors of flat functions
template<class T>
using FunctionVec = std::vector<DiscreteFunction<T>>;
/// \brief typedef for vectors of (flat) reward functions
typedef FunctionVec<Reward> RFunctionVec;

} // namespace crl

namespace gurobi {

/**
 * \brief A class that abstracts a Gurobi LP
 * All specific LP solver dependencies should be restricted to this hpp/cpp file
 * TODO: could implement iterative constraint generation if performance becomes prohibitive
 */
class _LP {
private:
  const crl::Domain _domain;
  /// \brief Storage for functions generated during variable elimination
  crl::FunctionSet<crl::Reward> F;
  /// \brief The Gurobi LP
  boost::shared_ptr<GRBEnv> _env;
  boost::shared_ptr<GRBModel> _lp;
  /// \brief The w variables added to this LP
  std::vector<GRBVar> _wvars;
public:
  /// \brief ctor
  _LP(const crl::Domain& domain)
  : _domain(domain), F(domain) { }
  /// \brief dtor
  ~_LP() { }

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

/// \brief A Gurobi version of the LP example that ships with lpsolve
/// Source: http://lpsolve.sourceforge.net/5.5/formulate.htm
int lp_demo();

} // namespace testing

} // namespace gurobi

#endif /*ALP_GUROBI_HPP_*/
