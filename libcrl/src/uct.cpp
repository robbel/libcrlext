#include <cmath>
#include <iostream>
#include "cpputil.hpp"
#include "crl/uct.hpp"

using namespace std;
using namespace cpputil;
using namespace crl;

#define LOGUCT 0

// -------------------------------------------------------------------------------
// Data structures for the Q-table (tree).

void _UCTQTable::Add2Tree(hash_key_type hash_s, hash_key_type hash_a){
	if( !inTree(hash_s)) {
		// After the first access the default constructor will be called.
		(*this)[hash_s].N = 0; // We do not update anything here (Call UpdateModel).
	}
	if ( !inTree(hash_s, hash_a)) {
		// The same for action entry, after the first call the default object will be created for given key.
		(*this)[hash_s][hash_a].Ni = 0; // also no data updates
	}
}

bool _UCTQTable::inTree(hash_key_type hash_s, hash_key_type hash_a) const {
	_UCTQTable::const_iterator state = this->find(hash_s);
	if(state != this->end()){
		_CStateInfoHash::const_iterator action = state->second.find(hash_a);
		if(action != state->second.end()){
			return true;
		}
	}
	return false;
}

bool _UCTQTable::inTree(hash_key_type hash_s) const {
	_UCTQTable::const_iterator state = this->find(hash_s);
	if(state != this->end()){
		return true;
	}
	return false;
}

hash_key_type _UCTQTable::getBestActionHash(hash_key_type& hash_s) {
	_CStateInfoHash& state_info = (*this)[hash_s];
	Reward max = INT_MIN;
	hash_key_type max_hash_a = 0;
	assert(!state_info.empty() && "do not call this methods when there is no action of hash_s in the Q-table");
	for (_CStateInfoHash::const_iterator iter = state_info.begin(); iter != state_info.end(); iter++) { // for all actions
		 Reward current = iter->second.getValue();
		 if (current > max) {
			 max = current;
			 max_hash_a = iter->first;
		 }
	}
	return max_hash_a;
}

// -------------------------------------------------------------------------------
// The UCT planner

void _UCTPlanner::runSimulation(const State& s) {
	CTraceList trace; // trace
	int steps=0;
	Reward start_metric = 0;	// Reward metric when we start (we start with 0 here and add up everything encountered on the trace  during sampling).
	Reward current_metric = 0;	// Reward metric when we start (we start with 0 here and add up everything encountered on the trace  during sampling).
	const State& current_state = s;

	// Sampling one trajectory until the goal state is reached.
	bool goal_is_reached = false;
	do{
		hash_key_type current_state_hash = current_state.getIndex();
		ActionIterator curr_actions_iter = _mdp->A(current_state);
		ActionList curr_actions(curr_actions_iter);
		hash_key_type selected_action;
		Action action;
		if (!_qtable.inTree(current_state_hash)) { // If not in the tree just pick up one action heuristically.
			action = GetHuristicAction(curr_actions);
			selected_action = action.getIndex();
		} else {
			size_t an = curr_actions.size();
			vector<hash_key_type> temp_hashes;
			temp_hashes.resize(an);
			for(size_t ai=0; ai<an; ai++){
				temp_hashes[ai] = curr_actions[ai].getIndex();
			}
			action = GetUCTAction(current_state_hash , curr_actions, temp_hashes, selected_action);
		}

		State new_state;
		try{
			new_state = _mdp->T(current_state,action)->sample();
		}
		catch (DistributionException e) {
			// sampling throws an exception if all next states have probability 0.
			goal_is_reached = true;
		}
		Reward step_reward = _mdp->R(current_state,action);

		hash_key_type new_state_hash = new_state.getIndex();
		if (!goal_is_reached) {
			trace.push_back( CTraceData(current_state_hash, selected_action, step_reward) );
		}

		Reward new_metric = current_metric + step_reward;
		current_metric = new_metric;
		if (_fullTree && !goal_is_reached) {
			// Add immediately so it can influence exploration within this episode.
			_qtable.Add2Tree(current_state_hash, selected_action);
			UpdateModel(current_state_hash, selected_action, new_state_hash, step_reward);
		}
	}while( (steps++ < _maxSteps || _maxSteps == 0) && !goal_is_reached);

	Reward cumulative_reward = current_metric - start_metric; // Cumulative reward.
	if (!goal_is_reached) {
		cout << "the goal state has not been reached, penalising the trace !!!" << endl;
		cumulative_reward += -100; // if the steps limit exceeded then penalise this trajectory.
	}

	DoBackups(trace, cumulative_reward);
}

Action _UCTPlanner::GetUCTAction(hash_key_type state_hash, ActionList& actions, vector<hash_key_type>& hash_list, hash_key_type& hash_a, bool explore) {
	assert(actions.size() == hash_list.size());
	if (!_qtable.inTree(state_hash)) { // State not in the tree, choose random action.
		size_t i = size_t(rand()/(RAND_MAX + 1.0)*actions.size());
		hash_a = hash_list[i];
		return actions[i];
	}
	_CStateInfoHash& state_info = _qtable[state_hash]; // we are sure here that state exists in the map.
	double logN = log(double(state_info.N));
	size_t an = hash_list.size();
	multimap<double,hash_key_type> pq; // pq of pairs <Vi, action's id in the list>
	for (size_t t=0; t<an; t++){
		if (!state_info.inTree(hash_list[t])) {
			if (explore) {
				pq.insert(pair<double,hash_key_type>(INT_MAX, hash_list[t]));	// not attempted actions have very high valus.
			}
		}else{
			_CActionInfoHash& action_info = state_info[hash_list[t]]; // for sure action t is in the tree
			if (action_info.Ni>0) {
				double Vi;
				Vi = double(action_info.Wi) / (double)action_info.Ni;
				if (explore) { // we do not add interval to during exploitation for the final action choice.
					double conf_interv = (2 * _rmax) * sqrt(logN / double(action_info.Ni)); // C=2*Rmax
					Vi += conf_interv;
				}
				pq.insert(pair<double,hash_key_type>(Vi,hash_list[t]));
			}else{
				if (explore) {
					pq.insert(pair<double,hash_key_type>(INT_MAX,hash_list[t]));
				}
			}
		}
	}
	if (LOGUCT > 0) {// || state_hash==575785) {
		cout << "making decision:" << endl;
		for (multimap<double,hash_key_type>::const_iterator citer=pq.begin(); citer!=pq.end(); citer++) {
			int j;
			for (int i=0; i<int(actions.size()); i++) {
				if (actions[i].getIndex() == (*citer).second) {
					j=i;
					break;
				}
			}
			cout << (*citer).first << " " << (*citer).second << " "<< actions[j] << "," << actions[j] << endl;
		}
		cout << endl;
	}
	multimap<double,hash_key_type>::iterator last = pq.end(); last--;
	double max = last->first;
	double n = pq.count(max); // how many elements in pq is equal to the max
	if(n==1){ // only one max
		int a_index = -1;
		for(int ah = 0; ah < int(hash_list.size()); ah++) {
			if (hash_list[ah] == last->second) {
				a_index = ah;
				break;
			}
		}
		hash_a = hash_list[a_index];
		return actions[a_index];
	}
	multimap<double,hash_key_type>::iterator itlow	=	pq.lower_bound(max);
	//multimap<int,int>::iterator itup	=	pq.upper_bound (max);
	int i = int( double(rand()) / (double(RAND_MAX) + 1.0) * n );
	int a_index=-1;
	for (int t=0; t<n; t++, itlow++){
		if (t==i){
			a_index = t;
			hash_a = itlow->second;
			break;
		}
	}
	return actions[a_index];
}

Action _UCTPlanner::getAction(const State& s) {
	// (*) initialise
	Size num_runs = 0;
	time_t start_time = time_in_milli();
	time_t time_total = 0;
	if (_clear_tree) {
		ClearTree();
	}
	ActionIterator aitr = _mdp->A(s);
	ActionList alist(aitr);
	assert(!alist.empty() && "there are no more actions in current state");
	if (alist.size() == 1) {
		return alist.front(); // do not plan if there is only one action available.
	}

	// (*) loop over simulations
	while ((_run_limit==0 || num_runs<_run_limit) && //0 for limit means unlimited
          (_time_limit==0 || time_total<_time_limit)) {

		runSimulation(s);

		time_total = time_in_milli()-start_time;
		num_runs++;
	}

	// (*) return the best action without bonuses
	hash_key_type hash_a = _qtable.getBestActionHash((hash_key_type&)(s.getIndex()));
	aitr->reset();
	while (aitr->hasNext()) {
		Action fa = aitr->next();
		if (fa.getIndex() == hash_a) {
			return fa;
		}
	}
	assert(!"ERROR: we cannot be here, there is no action with selected hash_a");
}

void _UCTPlanner::UpdateModel(hash_key_type hash_s, hash_key_type hash_a, hash_key_type hash_sprime, double reward){
	if (_qtable.inTree(hash_s,hash_a)) {
		_qtable[hash_s].N++;
		_CActionInfoHash& qsa = _qtable[hash_s][hash_a];
		qsa.Ni++;
	}
}

void _UCTPlanner::DoBackups(CTraceList& trace, double cumulative_reward) {
	// (*)
	if (_reward_type == 1) { // compute MC reward from each state
		for( int i = int(trace.size() - 2); i >= 0; i-- ) {
			trace[i].reward += trace[i+1].reward;
		}
	}

	// (*)
	if (_fullTree) { // here the entire trace is in the Q-table
		for( int i = 0; i < int(trace.size()); i++ ) { /// The goal state is not in the trace.
			_CActionInfoHash &qsa = _qtable[trace[i].hash_s][trace[i].hash_a];
			if (_reward_type == 0) {
				qsa.Wi += cumulative_reward;	// final game reward
			} else {
				qsa.Wi += trace[i].reward;	// cost to go reward
			}
		}
	} else { // extending the tree by one state form the trace.
		for( int i = 0; i < int(trace.size()); i++ ) { /// The goal state is not in the trace.
			if ( _qtable.inTree(trace[i].hash_s, trace[i].hash_a) ) {
				_CActionInfoHash &qsa = _qtable[trace[i].hash_s][trace[i].hash_a];
				if (_reward_type == 0) {
					qsa.Wi += cumulative_reward;	// final game reward
				} else {
					qsa.Wi += trace[i].reward;	// cost to go reward
				}
			} else { // add one and break
				_qtable.Add2Tree(trace[i].hash_s, trace[i].hash_a);
				UpdateModel(trace[i].hash_s, trace[i].hash_a, 0, trace[i].reward);
				_CActionInfoHash &action_info = _qtable[trace[i].hash_s][trace[i].hash_a];
				if (_reward_type == 0) {
					action_info.Wi = cumulative_reward;
				} else {
					action_info.Wi = trace[i].reward;
				}
				return;
			}
		}
	}
}

_UCTPlanner::_UCTPlanner(Domain domain, MDP mdp, Reward gamma)
	: _domain(domain), _qtable(domain), _mdp(mdp), _gamma(gamma), _run_limit(0), _time_limit(0) {
	_clear_tree = false;
	_fullTree = true;
	_reward_type = 1;
	_rmax = 100; // TODO: we need to be able to get this information from _mdp
	_maxSteps = 0;
}

_FlatUCTPlanner::_FlatUCTPlanner(Domain domain, MDP mdp, Reward gamma)
	: _UCTPlanner(domain, mdp, gamma) {
}

Size iterator_size(ActionIterator& actions) {
	actions->reset();
	Size n = 0;
	while (actions->hasNext()) {
		actions->next();
		n++;
	}
	return n;
}



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

/*

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
*/
