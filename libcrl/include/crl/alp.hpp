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
#include "crl/function.hpp"
#include "crl/factor_learner.hpp"

namespace crl {

/**
 * A planner that uses an approximate linear program (ALP) along with a factored value function
 * \see Guestrin, Koller, Parr, and Venkatarman 2003
 */
class _ALPPlanner : public _Planner {
protected:
  /// \brief The factored dynamics model
  FactoredMDP _mdp;
public:
  _ALPPlanner() { }


};

} // namespace crl

#endif
