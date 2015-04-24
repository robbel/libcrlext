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

//
// todo
//

/**
 * \brief A class that abstracts a Gurobi LP
 * \todo Just a stub for now..
 * All specific LP solver dependencies should be restricted to this hpp/cpp file
 */
class _LP {
private:
  const crl::Domain _domain;
  /// \brief Storage for functions generated during variable elimination
  crl::FunctionSet<crl::Reward> F;
public:
  /// \brief ctor
  _LP(const crl::Domain& domain)
  : _domain(domain), F(domain) { }
  /// \brief dtor
  ~_LP();

  /// \brief Given targets \f$C\f$ and \f$\mathbf{b}\f$ compute polynomial set of constraints
  /// \return 0 iff successful
  int generateLP(const crl::RFunctionVec& C, const crl::RFunctionVec& b, const crl::SizeVec& elim_order);
  /// \brief Solve this LP
  /// \return 0 iff successful
  int solve(const std::vector<double>& alpha, crl::_FactoredValueFunction* vfn);
};
typedef boost::shared_ptr<_LP> LP;

namespace testing {

//
// todo
//

} // namespace testing

} // namespace gurobi

#endif /*ALP_GUROBI_HPP_*/
