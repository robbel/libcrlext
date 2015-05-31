/*
    Copyright 2015 Philipp Robbel
    Copyright 2008 John Asmuth
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef GLUE_BIGENV_HPP_
#define GLUE_BIGENV_HPP_

#include <rlglue/RL_common.h>
#include <crl/crl.hpp>
#include "crl/bigindex.hpp"

//
// glue-util replacement
//

///
/// \brief For interfacing with rl-glue: populate an \a observation_t from \a crl::BigState
/// \see glue_bigenv.cpp in graphprop library
///
void populateBigState(crl::Domain domain, crl::BigState s, observation_t* obs);

///
/// \brief For interfacing with rl-glue: return an \a crl::Action from an \a action_t
/// \see glue_env.cpp in glue-crl library
///
crl::Action getAction(crl::Domain domain, const action_t* act);

//
// glue-crl replacement for BigEnvironment
//

namespace crl {

///
/// \brief Construct and obtain domain corresponding to desired crl::BigEnvironment
/// \see env_init()
///
crl::Domain getCRLEnvironmentDomain();

///
/// \brief Construct and obtain desired \a crl::BigEnvironment from a \a crl::Domain
/// \see env_init()
///
crl::BigEnvironment getCRLBigEnvironment(crl::Domain domain);

} // namespace crl

#endif /*GLUE_BIGENV_HPP_*/
