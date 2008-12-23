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
 
#include <math.h>
#include <iostream>
#include <cpputil.hpp>
#include "crl/uct.hpp"

using namespace std;
using namespace cpputil;
using namespace crl;

_UCTPlanner::_UCTPlanner(const MDP& mdp, float confidence_coeff, float gamma)
: _mdp(mdp), _confidence_coeff(confidence_coeff), _gamma(gamma),
  _time_limit(-1), _run_limit(10), _learning_rate(0),
  _frequent_states(new _StateSet()) {
  	
}

Reward _UCTPlanner::getConfidence(const State& s, const Action& a, Size depth) {
	int s_visits = getVisits(s, depth)+1;
	int sa_visits = getVisits(s, a, depth)+1;
	Reward conf = 2*_confidence_coeff*sqrt(log(s_visits)/sa_visits);
	return conf;
}

Action _UCTPlanner::selectAction(ActionIterator& aitr, const State& s, Size depth) {
	Action selected_action;
	Reward selected_qc_val = -1*std::numeric_limits<float>::max();
	while (aitr->hasNext()) {
		Action a = aitr->next();
		
		Reward q = getQ(s, a, depth);
		Reward c = getConfidence(s, a, depth);
		Reward qc_val = q+c;
		
		if (qc_val>selected_qc_val) {
			selected_qc_val = qc_val;
			selected_action = a;
		}
	}
	return selected_action;
}

Reward _UCTPlanner::runSimulation(const State& s, Size depth) {
	if (isTerminal(s, depth)) {
		cerr << "terminal depth = " << depth << endl;
		return 0;
	}
	
	ActionIterator aitr = _mdp->A(s);
	if (!aitr->hasNext()) {
		return 0;
	}
		
	Action selected_action = selectAction(aitr, s, depth);
	incrVisits(s, selected_action, depth);

	Size s_visits = getVisits(s, depth);
	Size frequency_threshold = 25;
	if (s_visits == frequency_threshold) {
		_frequent_states->insert(s);
		_ps_planner->insert(s, getQ(s, selected_action, depth));
	}

	Observation o = _mdp->sample(s, selected_action);
	State n = o->getState();
	Reward r = o->getReward();

	Reward tail_val = runSimulation(n, depth+1);
	Reward q = r + _gamma*tail_val;
	
	if (s_visits >= frequency_threshold) {
		Reward error = 0;
		if (_learning_rate == 0) {
			error = fabs(getQ(s, selected_action, depth)-q); 
			avgQ(s, selected_action, q, depth);	
		}
		else {
			Reward old_q = getQ(s, selected_action, depth);
			Reward new_q = (1-_learning_rate)*old_q + _learning_rate*q;
			setQ(s, selected_action, new_q, depth);
			error = fabs(old_q-new_q);
		}
		
		_ps_planner->insert(s, error);
	}

	return q;
}
void _UCTPlanner::avgQ(const State& s, const Action& a, Reward q, Size depth) {
	Reward old_q = getQ(s, a, depth);
	int count = getVisits(s, a, depth);
	Reward new_q = old_q + (q-old_q)/count;
	setQ(s, a, new_q, depth);
}

Action _UCTPlanner::getAction(const State& s) {
	int runs = 0;
	time_t start_time = time_in_milli();
	time_t time_total = 0; 
	while ((_run_limit==-1 || runs<_run_limit) &&
	       (_time_limit==-1 || time_total<_time_limit)) {
		runSimulation(s);
		runs++;
		time_total = time_in_milli()-start_time;
	}

	if (_ps_planner) {
//		time_t vi_start_time = time_in_milli();
		StateIterator sitr(new _StateSetIterator(_frequent_states));
		_ps_planner->sweep();
//		cerr << time_in_milli() - vi_start_time << endl;
	}
	
	Action a = getQTable()->getBestAction(s);
	cerr << s << " -> " << a << endl;
	return a;
}
QTable _UCTPlanner::getQTable() {
	return getQTable(0);
}
void _UCTPlanner::setTimeLimit(time_t time_limit) {
	_time_limit = time_limit;
}
void _UCTPlanner::setRunLimit(int run_limit) {
	_run_limit = run_limit;
}
void _UCTPlanner::setConfidenceCoeff(float confidence_coeff) {
	_confidence_coeff = confidence_coeff;
}
void _UCTPlanner::setLearningRate(float learning_rate) {
	_learning_rate = learning_rate;
}

bool _FactoredUCTPlanner::isTerminal(const State& s, Size depth) {
//	int visits = getVisits(s, depth);
	float term_decay = .9;
	float term_factor = .001;
	float term_offset = .1;
	float prob = 1-pow(term_decay, -1*term_factor*((depth+1)+term_offset));
	return randDouble() < prob;//pow(0.99, visits+1);
//	return depth>50;
}
int _FactoredUCTPlanner::getVisits(const State& s, Size depth) {
	depth = 0;
	const vector<Size>& v = _s_visits.getValue(s);
	if (v.size() <= depth)
		return 0;
	return v[depth];
}
int _FactoredUCTPlanner::getVisits(const State& s, const Action& a, Size depth) {
	depth = 0;
	const vector<Size>& v = _sa_visits.getValue(s, a);
	if (v.size() <= depth)
		return 0;
	return v[depth];
}
void _FactoredUCTPlanner::setQ(const State& s, const Action& a, Reward q, Size depth) {
	depth = 0;
	if (_qtables.size() <= depth)
		_qtables.resize(depth+1);
	if (!_qtables[depth])
		_qtables[depth] = FQTable(new _FQTable(_domain));
	_qtables[depth]->setQ(s, a, q);
}
const Reward _FactoredUCTPlanner::getQ(const State& s, const Action& a, Size depth) {
	depth = 0;
	if (_qtables.size() <= depth)
		_qtables.resize(depth+1);
	if (!_qtables[depth])
		_qtables[depth] = FQTable(new _FQTable(_domain));
	return _qtables[depth]->getQ(s, a);
}

_FactoredUCTPlanner::_FactoredUCTPlanner(const Domain& domain, const MDP& mdp, float confidence_bias, float gamma)
: _UCTPlanner(mdp, confidence_bias, gamma),
  _domain(domain), _sa_visits(_domain), _s_visits(_domain) {
	SCountTable heap_indices = SCountTable(new _FStateTable<Index>(domain, -1));
	SPriorityQueue pqueue = SPriorityQueue(new _SPriorityQueue(heap_indices));
	_ps_planner = PSPlanner(new _PSPlanner(_mdp, .0001, _gamma, getQTable(0), pqueue));
}
QTable _FactoredUCTPlanner::getQTable(Size depth) {
	depth = 0;
	if (_qtables.size() <= depth)
		_qtables.resize(depth+1);
	if (!_qtables[depth])
		_qtables[depth] = FQTable(new _FQTable(_domain, 0));
	return _qtables[depth];
}
void _FactoredUCTPlanner::incrVisits(const State& s, const Action& a, Size depth) {
	depth = 0;
	vector<Size>& sv = _s_visits.getValue(s);
	if (sv.size() <= depth)
		sv.resize(depth+1);
	sv[depth] = getVisits(s, depth)+1;
	
	vector<Size>& sav = _sa_visits.getValue(s, a);
	if (sav.size() <= depth)
		sav.resize(depth+1);
	sav[depth] = getVisits(s, a, depth)+1;
}
