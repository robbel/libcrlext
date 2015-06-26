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
#include <boost/functional/hash.hpp>
#include "crl/common.hpp"

namespace crl {

///
/// \brief A `lifted' factor comprising a set of (state) variables
/// \see LiftedCounter
/// \todo compare with performance of unordered_set
///
class _LiftedFactor {
protected:
  /// \brief The state factors relevant for this lifted factor, indicated by their absolute position in the global domain
  SizeVec _state_dom;
  /// \brief Hash of _state_dom for comparison to other LiftedFactor scopes
  std::size_t _dom_hash;
public:
  /// \brief ctor
  _LiftedFactor(std::initializer_list<Size> il)
  : _state_dom(std::move(il)) {
    std::sort(_state_dom.begin(), _state_dom.end());
    _dom_hash = boost::hash_range(_state_dom.begin(), _state_dom.end());
  }
  /// \brief True iff state factor i is included in the scope of this lifted operation
  bool containsStateFactor(Size i) const {
    return std::binary_search(_state_dom.begin(), _state_dom.end(), i); // sorted assumption
  }
  /// \brief Erase state factor `i' from the scope of this function
  void eraseStateFactor(Size i);
  /// \brief The state factor indices (w.r.t. global \a Domain) relevant for this lifted operation
  const SizeVec& getStateFactors() const {
    return _state_dom;
  }
  /// \todo Return the \a FactorRange corresponding to this lifted operation
  /// \see Range::getSpan()
  FactorRange getRange() const {
    return FactorRange(0,_state_dom.size());
  }
  /// \brief Return hash of this factor's domain
  std::size_t getHash() const {
    return _dom_hash;
  }

  /// \brief operators
  bool operator==(const _LiftedFactor& other) const {
    return _dom_hash == other._dom_hash;
  }
  bool operator<(const _LiftedFactor& other) const {
    return _dom_hash < other._dom_hash;
  }
};
typedef boost::shared_ptr<_LiftedFactor> LiftedFactor;
typedef std::vector<LiftedFactor> LiftedVec;

///
/// \todo A lifted counter is a function that summarizes how many state factors in its scope are `enabled'
/// Defined as #{A,B,C}->R where (for example) #{A=5,B=3,C=1}->3, #{0,0,0}->0, and #{1,0,1}->2.
///
class _LiftedCounter : public _LiftedFactor {
protected:
public:
  /// \brief ctor
  _LiftedCounter(std::initializer_list<Size> il)
  : _LiftedFactor(std::move(il)) { }
};
typedef boost::shared_ptr<_LiftedCounter> LiftedCounter;

} // namespace crl

#endif /*LIFTED_OPS_HPP_*/
