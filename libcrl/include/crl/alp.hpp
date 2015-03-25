/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef ALP_HPP_
#define ALP_HPP_

#include <iostream>
#include "crl/common.hpp"

namespace crl {

/**
 * \brief The abstract interface for a basis function defined over a subset of state variables.
 */
template<class T>
class Basis {
protected:
  /// \brief The domain which includes all state and action factors
  const Domain _domain;
  /// \brief The (subset of) state factors relevant for this basis function
  SizeVec _state_dep;
public:
  /// \brief ctor
  Basis(const Domain& domain)
  : _domain(domain) { }
  /// \brief dtor
  virtual ~Basis() { }

  /// \brief
  void addStateFactor(Size i) {
    assert(i < _domain->getNumStateFactors());
    _state_dep.push_back(i);
  }

  ///
  /// \brief evaluate the basis function
  ///
  virtual T compute(std::initializer_list<Factor> fac_val) const = 0;
  /// \brief convenience function for evaluating this basis function
  virtual T operator()(std::initializer_list<Factor> fac_val) const {
    return compute(std::move(fac_val));
  }
};

} // namespace crl

#endif
