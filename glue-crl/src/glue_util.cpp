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

#include <rlglue/utils/C/RLStruct_util.h>
#include <rlglue/utils/C/TaskSpec_Parser.h>
#include <crl/crl.hpp>

using namespace crl;

void populateState(Domain domain, State s, observation_t* obs) {
	for (Size i=0; i<domain->getNumStateFactors(); i++) {
		obs->intArray[i] = s.getFactor(i);
	}
}
void populateAction(Domain domain, Action a, action_t* act) {
	for (Size i=0; i<domain->getNumActionFactors(); i++) {
		act->intArray[i] = a.getFactor(i);
	}
}

State getState(Domain domain, const observation_t* obs) {
	State s(domain);
	for (Size i=0; i<domain->getNumStateFactors(); i++) {
		s.setFactor(i, obs->intArray[i]);
	}
	return s;
}
Action getAction(Domain domain, const action_t* act) {
	Action a(domain);
	for (Size i=0; i<domain->getNumActionFactors(); i++) {
		a.setFactor(i, act->intArray[i]);
	}
	return a;
}
