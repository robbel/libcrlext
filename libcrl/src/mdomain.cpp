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
#include <boost/shared_ptr.hpp>
#include "crl/mdomain.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

Action _MQTable::getBestAction(const State& s) {
	MapKeyIterator<Action,ARMap> kitr(_q_map[s]);
	if (!kitr.hasNext())
		return _null_action;
	Action best_action = kitr.next();
	Reward best_q = getQ(s, best_action);
	while (kitr.hasNext()) {
		Action a = kitr.next();
		Reward q = getQ(s, a);
		if (q>best_q) {
			best_q = q;
			best_action = a;
		}
	}
	return best_action;
}

void _MapMDP::setT(const State& s, const Action& a, const State& n, Probability p) {
	//cerr << "_FlatMDP::setT(" << s << "," << a << "," << n << "," << p << ")" << endl;
	_action_map[s].insert(a);
	_known_states.insert(s);
	_known_actions.insert(a);
	
	if (_T_map[s].find(a) == _T_map[s].end()) {
		_T_map[s][a] = StateDistribution(new _MStateDistribution());
	}
	MStateDistribution dist(
	  boost::shared_polymorphic_downcast<_MStateDistribution>(_T_map[s][a]));
	dist->setP(n, p);
}

void _MapMDP::setR(const State& s, const Action& a, Reward r) {
	_known_states.insert(s);
	_known_actions.insert(a);
	_R_map[s][a] = r;
}

void _MapMDP::clear(const State& s, const Action& a) {
	if (_T_map[s].find(a) == _T_map[s].end())
		return;
	MStateDistribution dist(
	  boost::shared_polymorphic_downcast<_MStateDistribution>(_T_map[s][a]));
	dist->clear();
}

void _MapMDP::clear() {
	_known_states.clear();
	_known_actions.clear();
	_T_map.clear();
	_R_map.clear();
}
