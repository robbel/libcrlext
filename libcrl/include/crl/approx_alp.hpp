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
public:
  /// \brief ctor
  _ApproxALP(ALPPlanner alp)
  : _alp(std::move(alp)) { }

  /// \brief Build the \a dai::FactorGraph corresponding to functions \f$C\f$ and \f$\mathbf{b}\f$ in the ALP
  /// \note This uses the weight vector already computed by the ALP
  void buildFactorGraph();

};
typedef boost::shared_ptr<_ApproxALP> ApproxALP;

namespace testing {

/// \brief Basic libDAI max-plus test inspired by libdai's example.cpp
std::vector<size_t> maxplus_demo();

} // namespace testing

#endif /*defined(DAI_WITH_BP) && defined(DAI_WITH_JTREE)*/

} // namespace crl

#endif /*APPROX_ALP_HPP_*/
