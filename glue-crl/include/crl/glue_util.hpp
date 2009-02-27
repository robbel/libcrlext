/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth

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
#include <crl/fdomain.hpp>

void populateState(crl::Domain domain, crl::State s, observation_t* obs);
void populateAction(crl::Domain domain, crl::Action a, action_t* act);
//crl::State getState(crl::Domain domain, const observation_t* obs);
crl::Action getAction(crl::Domain domain, const action_t* act);

#endif /*GLUE_UTIL_HPP_*/
