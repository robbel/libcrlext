/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth

    This file is part of CRL.

    CRL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CRL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CRL.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include <iostream>
#include <cpputil.hpp>
#include "crl/environment.hpp"

using namespace std;
using namespace cpputil;
using namespace crl;

_MDPEnvironment::_MDPEnvironment(const MDP& mdp, const State& start_state)
: _mdp(mdp), _start_state(start_state) {
	
}

State _MDPEnvironment::begin() {
	_current_state = _start_state;
	return _current_state;
}

bool _MDPEnvironment::isTerminated() {
	return !(_mdp->A(_current_state)->hasNext());
}

Observation _MDPEnvironment::getObservation(const Action& a) {
	Observation o = _mdp->sample(_current_state, a);
	cout << _current_state << " x " << a << " -> " << o << endl;
	_current_state = o->getState();
	return o;
}
