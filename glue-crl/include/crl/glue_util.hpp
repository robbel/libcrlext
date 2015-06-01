/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth
    Copyright 2015 Philipp Robbel

    This file is part of CRL:RL-Glue.

    CRL:RL-Glue is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CRL:RL-Glue is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CRL:RL-Glue.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef GLUE_UTIL_HPP_
#define GLUE_UTIL_HPP_

#include <rlglue/RL_common.h>
#include <crl/crl.hpp>

///
/// \brief For interfacing with rl-glue: populate an \a observation_t from \a crl::State
/// \see glue_env.cpp in glue-crl library
///
void populateState(crl::Domain domain, const crl::State& s, observation_t* obs);

///
/// \brief For interfacing with rl-glue: populate an \a action_t from \a crl::Action
/// \see glue_agent.cpp in glue-crl library
///
void populateAction(crl::Domain domain, crl::Action a, action_t* act);

///
/// \brief For interfacing with rl-glue: return an \a crl::Action from an \a action_t
/// \see glue_env.cpp in glue-crl library
///
crl::Action getAction(crl::Domain domain, const action_t* act);
//crl::State getState(crl::Domain domain, const observation_t* obs);

#endif /*GLUE_UTIL_HPP_*/
