/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth
    Copyright 2015 Philipp Robbel

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
#include "crl/crl.hpp"
#include "crl/flat_tables.hpp"

using namespace std;
using namespace crl;

_Domain::_Domain() {
	_num_states = 1; // there's always the all-zeros state
	_num_actions = 1;
}
void _Domain::addStateFactor(Factor min, Factor max, string name) {
	FactorRange r(min, max);
	_state_ranges.push_back(r);
	_state_index_components.push_back(_num_states);
	// detect unsigned `overflow' (i.e., modulo) in large domains
	Size num_states = _num_states;
	_num_states *= r.getSpan()+1;
	if(_num_states < num_states) {
	  _num_states = 0;
	}
	_state_names.push_back(name);
}
void _Domain::addActionFactor(Factor min, Factor max, string name) {
	FactorRange r(min, max);
	_action_ranges.push_back(r);
	_action_index_components.push_back(_num_actions);
	_num_actions *= r.getSpan()+1;
	_action_names.push_back(name);
}

void RLType::print(std::ostream& os) const {
	for (Size i=0; i<size(); i++) {
		if (i != 0)
			os << " ";
		os << getFactor(i);
	}
}

DistributionException::DistributionException(std::string what)
: Exception("DistributionException", what) {

}

void _QTable::print(std::ostream& os, StateIterator sitr, ActionIterator aitr) {
	sitr->reset();
	while (sitr->hasNext()) {
		const State& s = sitr->next();
		os << "V(" << s << ") = " << getV(s) << endl;
		aitr->reset();
		while (aitr->hasNext()) {
			const Action& a = aitr->next();
			os << " Q(" << a << ") = " << getQ(s, a) << endl;
		}
	}
}

void _MDP::printXML(std::ostream& os) {
	os << "<MDP>" << endl;
	StateIterator sitr = S();
	while (sitr->hasNext()) {
		const State& s = sitr->next();
		os << " <State desc=\"" << s << "\">" << endl;
		ActionIterator aitr = A(s);
		while (aitr->hasNext()) {
			const Action& a = aitr->next();
			os << "  <Action desc=\"" << a << "\" reward=\"" << R(s, a) << "\">" << endl;
			StateDistribution sd = T(s, a);
			StateIterator nitr = sd->iterator();
			while (nitr->hasNext()) {
				const State& n = nitr->next();
				Probability p = sd->P(n);
				os << "   <State desc=\"" << n << "\" probability=\"" << p << "\"/>" << endl;
			}
			os << "  </Action>" << endl;
		}
		os << " </State>" << endl;
	}
	os << "</MDP>" << endl;
}

_Agent::_Agent(const Domain& domain, Planner planner)
: _planner(planner), _last_state(domain->isBig() ? stateImpl._big_s : stateImpl._s) {

}
_Agent::_Agent(const Domain& domain, Planner planner, Learner learner)
: _planner(planner), _learner(learner), _last_state(domain->isBig() ? stateImpl._big_s : stateImpl._s) {

}
void _Agent::begin(const State& s) {
	_last_state = s;
}
void _Agent::end() {

}
bool _Agent::observe(const Observation& o) {
	bool learned = false;
	if (_learner)
		learned = _learner->observe(_last_state, _last_action, o);
	_last_state = o->getState();
	return learned;
}
Action _Agent::getAction(const State& s) {
	_last_action = _planner->getAction(s);
	return _last_action;
}
