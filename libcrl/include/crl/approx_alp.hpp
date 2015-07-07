/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef APPROX_ALP_HPP_
#define APPROX_ALP_HPP_

#include <iostream>
#include <dai/alldai.h>  // Include main libdai header
#include <dai/bp.h>
#include "crl/alp.hpp"

namespace crl {

//
// Approximate inference support for ALP constraint generation
//

#if defined(DAI_WITH_BP)

/**
 * The ApproxALP is an iterative constraint generation method for computing useful (i.e., in apprximation,
 * maximally violated) constraints.
 * \note Requires the max-plus implementation for libdai maintained in ./external/libdai.git.patch
 */
class _ApproxALP {
protected:
  /// \brief The ALPPlanner used for automatic constraint generation
  ALPPlanner _alp;
  /// \brief The set of dai::Var representing the (global) Domain
  std::vector<dai::Var> _vars;
  /// \brief The number of action variables in the (global) Domain
  const Size _nrActions;
  /// \brief The dai::FactorGraph corresponding to functions \f$C\f$ and \f$\mathbf{b}\f$ in the ALP
  /// \note Factors are sorted: first functions \f$C\f$, then \f$\mathbf{b}\f$
  boost::shared_ptr<dai::FactorGraph> _fg;
  /// \brief Stores backups of the original setting of functions \f$C\f$ (restored before setWeights() call)
  std::vector<dai::Factor> _backup;

  /// \brief Construct a dai::Factor corresponding to the crl::DiscreteFunction
  dai::Factor makeDAIFactor(const DiscreteFunction<Reward>& f);
  /// \brief Build the \a dai::FactorGraph corresponding to functions \f$C\f$ and \f$\mathbf{b}\f$ in the ALP
  void buildFactorGraph();
public:
  /// \brief Construct the dai::FactorGraph corresponding to functions \f$C\f$ and \f$\mathbf{b}\f$ in the ALP
  _ApproxALP(const Domain& domain, ALPPlanner alp);
  /// \brief Return the dai::FactorGraph
  boost::shared_ptr<dai::FactorGraph> getFactorGraph() const {
    return _fg;
  }

  /// \brief Update the weights associated with each Factor corresponding to \f$C\f$
  void setWeights(const std::vector<double> weights);
  /// \brief Run max-plus to obtain the approximately maximizing State and Action in the FactorGraph.
  /// \param[out] s The \a crl::State at which the (approximate) maximum is achieved
  /// \param[out] a The \a crl::Action at which the (approximate) maximum is achieved
  void approxArgmax(State& s, Action& a);
};
typedef boost::shared_ptr<_ApproxALP> ApproxALP;

namespace testing {

/// \brief Basic libDAI max-plus test inspired by libdai's example.cpp
std::vector<size_t> maxplus_demo();

} // namespace testing

#endif /*defined(DAI_WITH_BP)*/

} // namespace crl

#endif /*APPROX_ALP_HPP_*/
