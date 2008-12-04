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
#include "crl/crl.hpp"
#include "crl/fdomain.hpp"

using namespace std;
using namespace crl;

_Domain::_Domain() {
	_num_states = 1;
	_num_actions = 1;
}
void _Domain::addStateFactor(Factor min, Factor max) {
	FactorRange r(min, max);
	_state_ranges.push_back(r);
	_state_index_components.push_back(_num_states);
	_num_states *= r.getSpan()+1;	
}
void _Domain::addActionFactor(Factor min, Factor max) {
	FactorRange r(min, max);
	_action_ranges.push_back(r);	
	_action_index_components.push_back(_num_actions);
	_num_actions *= r.getSpan()+1;
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

void _MDP::printXML(std::ostream& os) {
	cout << "<MDP>" << endl;
	StateIterator sitr = S();
	while (sitr->hasNext()) {
		State s = sitr->next();
		cout << " <State desc=\"" << s << "\">" << endl;
		ActionIterator aitr = A(s);
		while (aitr->hasNext()) {
			Action a = aitr->next();
			cout << "  <Action desc=\"" << a << "\" reward=\"" << R(s, a) << "\">" << endl;
			StateDistribution sd = T(s, a);
			StateIterator nitr = sd->iterator();
			while (nitr->hasNext()) {
				State n = nitr->next();
				Probability p = sd->P(n);
				cout << "   <State desc=\"" << n << "\" probability=\"" << p << "\"/>" << endl;
			}
			cout << "  </Action>" << endl;
		}
		cout << " </State>" << endl;
	}
	cout << "</MDP>" << endl;
}

_Agent::_Agent(Planner planner)
: _planner(planner) {
	
}
_Agent::_Agent(Planner planner, Learner learner)
: _planner(planner), _learner(learner) {
	
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

_Experiment::_Experiment(const Environment& environment, const Agent& agent, int num_trials)
: _environment(environment), _agent(agent), _num_trials(num_trials) {
	
}
Reward _Experiment::runExperiment() {
	Reward total = 0;
	for (int trial=0; trial<_num_trials; trial++) {
//		cout << "******" << endl;
		State s = _environment->begin();
		_agent->begin(s);
		while (!_environment->isTerminated()) {
//			cout << " step" << endl;
			Action a = _agent->getAction(s);
			Observation o = _environment->getObservation(a);
//			cout << s << ", " << a << " -> " << o << endl;
			s = o->getState();
			total += o->getReward();
			_agent->observe(o);
		}
		_agent->end();
	}
	return total;
}
