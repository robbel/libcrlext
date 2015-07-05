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
 * \brief A class that abstracts a Gurobi LP.
 * All specific LP solver dependencies should be restricted to this hpp/cpp file.
 * Supports representation of the nonlinear max constraint in the ALP as well as iterative constraint addition.
 */
class _LP {
private:
  const crl::Domain _domain;
  /// \brief Storage for functions generated during variable elimination
  crl::FunctionSet<crl::Reward> _F;
  /// \brief The Gurobi LP
  boost::shared_ptr<GRBEnv> _env;
  boost::shared_ptr<GRBModel> _lp;
  /// \brief The w variables added to this LP
  std::vector<GRBVar> _wvars;

  /// \brief Set up LP with (only) the objective.
  void generateObjective(std::string name, const std::vector<double>& alpha);
  /// \brief Set up LP with the objective and abstract away functions \f$C\f$ and \f$\mathbf{b}\f$ with variables in the LP.
  /// \return A tuple of (0) a mapping from function \f$f\f$ to LP-variable offset for \f$f(\mathbf{x}_0,\mathbf{a}_0)\f$, and (1) the functions with empty scope
  /// \see generateLP, generateLiftedLP
  std::tuple<std::unordered_map<const _DiscreteFunction<Reward>*, int>, std::vector<DiscreteFunction<Reward>>>
  generateObjective(std::string name, const crl::RFunctionVec& C, const crl::RFunctionVec& b, const std::vector<double>& alpha);
  /// \brief Add a single constraint to this LP
  /// \note Requires previous set-up of LP, e.g. via \a generateObjective()
  GRBConstr addConstraint(const GRBLinExpr& lhs, char sense, const GRBLinExpr& rhs) {
    return _lp->addConstr(lhs, sense, rhs);
  }
  /// \brief Add a ALP constraint corresponding to \a State s and \a Action
  GRBConstr addStateActionConstraint(const State& s, const Action& a, const RFunctionVec& C, const RFunctionVec& b);
public:
  /// \brief ctor
  _LP(const crl::Domain& domain)
  : _domain(domain), _F(domain) { }
  /// \brief dtor
  ~_LP() { }

  /// \brief Given targets \f$C\f$ and \f$\mathbf{b}\f$ compute polynomial set of constraints
  /// \return 0 iff successful
  int generateLP(const crl::RFunctionVec& C, const crl::RFunctionVec& b, const std::vector<double>& alpha, const crl::SizeVec& elim_order);
  /// \brief Given targets \f$C\f$ and \f$\mathbf{b}\f$ compute polynomial set of constraints exploiting \a LiftedFactor efficiently
  /// \return 0 iff successful
  int generateLiftedLP(const crl::RFunctionVec& C, const crl::RFunctionVec& b, const std::vector<double>& alpha, const crl::SizeVec& elim_order);
  /// \brief Given targets \f$C\f$ and \f$\mathbf{b}\f$ compute possibly exponential set of constraints
  /// \return 0 iff successful
  int generateBLP(const crl::RFunctionVec& C, const crl::RFunctionVec& b, const std::vector<double>& alpha);
#if 0
  /// \brief Given basis functions in \f$C\f$ and \f$\mathbf{b}\f$ compute possibly exponential set of constraints
  /// \return 0 iff successful
  int generateVLP(const crl::RFunctionVec& C, const crl::RFunctionVec& b, const std::vector<double>& alpha, const _DBN& dbn, double gamma);
#endif
  /// \brief Given targets \f$C\f$ and \f$\mathbf{b}\f$ compute polynomial set of constraints (with L_1 and scope regularization)
  /// \return 0 iff successful
  int generateSCALP(const crl::RFunctionVec& C, const crl::RFunctionVec& b, const std::vector<double>& alpha, const crl::SizeVec& elim_order, double lambda, double beta, double ret_bound);

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
