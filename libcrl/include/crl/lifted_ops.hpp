/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef LIFTED_OPS_HPP_
#define LIFTED_OPS_HPP_

#include <iostream>
#include "crl/common.hpp"

namespace crl {

///
/// \brief A `lifted' operation on a set of (state) variables
/// \see LiftedCount
///
class LiftedOp {
protected:
  /// \brief The state factors relevant for this lifted operation, indicated by their absolute position in the global domain
  SizeVec _state_dom;
public:
  /// \brief True iff state factor i is included in the scope of this lifted operation
  bool containsStateFactor(Size i) const {
    return std::binary_search(_state_dom.begin(), _state_dom.end(), i); // sorted assumption
  }
  /// \brief The state factor indices (w.r.t. global \a Domain) relevant for this lifted operation
  const SizeVec& getStateFactors() const {
    return _state_dom;
  }
};

///
/// \todo A lifted counter is a function that summarizes how many state factors in its scope are `enabled'
/// Defined as #{A,B,C}->R where (for example) #{A=5,B=3,C=1}->3, #{0,0,0}->0, and #{1,0,1}->2.
///
class LiftedCount : public LiftedOp {
protected:
public:
  /// \todo Return the \a FactorRange corresponding to this lifted counter
  /// \see Range::getSpan()
  FactorRange getRange() const {
    return FactorRange(0,_state_dom.size());
  }

  // TODO: overload operator==
};

} // namespace crl

#endif /*LIFTED_OPS_HPP_*/
