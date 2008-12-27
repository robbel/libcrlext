/*
 * uctdata.cpp
 *
 *  Created on: 25-Dec-2008
 *      Author: Marek Grzes, University of York
 */

#include "crl/uctdata.hpp"

#include <cmath>
#include <map>

#define LOGUCT 1

using namespace std;

namespace crl{

// -----------------------------------------------------------------------------------
// _IUCTQTable Q-table

bool _IUCTQTable::inTree(Size hash_s, Size hash_a) const {
	HASH_MAP_STATE_INFO::const_iterator state = _qdata.find(hash_s);
	if(state != _qdata.end()){
		HASH_MAP_ACTIONS_INFO::const_iterator action = state->second->_actions.find(hash_a);
		if(action != state->second->_actions.end()){
			return true;
		}
	}
	return false;
}

bool _IUCTQTable::inTree(Size hash_s) const {
	HASH_MAP_STATE_INFO::const_iterator state = _qdata.find(hash_s);
	if(state != _qdata.end()){
		return true;
	}
	return false;
}

template <class S, class _S, class A, class _A>
void _IUCTQTable::Visit(Size hash_s, Size hash_a){
	if( !inTree(hash_s)) {
		S temp(new _S());
		temp->N = 1;
		_qdata[hash_s] = temp;
	}else{
		_qdata[hash_s]->N++;
	}
	if ( !inTree(hash_s, hash_a)) {
		A temp(new _A());
		temp->Ni=1;
		_qdata[hash_s]->_actions[hash_a] = temp;
	}else{
		_qdata[hash_s]->_actions[hash_a]->Ni++;
	}
}

// -----------------------------------------------------------------------------------
// _UCTQTable Q-table

Action 	_UCTQTable::getBestAction (const State &s) {
	CStateInfoHash& state_info = _qdata[s.getIndex()];
	Size best_action_hash;
	Reward max;
	bool first = true;
	for( HASH_MAP_ACTIONS_INFO::const_iterator iter = state_info->_actions.begin(); iter != state_info->_actions.end(); iter++) {
		const CActionInfoHash& action_info = iter->second;
		if (first) {
			best_action_hash = iter->first;
			max =  Reward( action_info->Vi / double(action_info->Ni) );
			first = false;
		} else {
			Reward current =  Reward( action_info->Vi / double(action_info->Ni) );
			if (current > max) {
				max = current;
				best_action_hash = iter->first;
			}
		}
	}
	return Action(_domain, best_action_hash);
}

// Standard UCT without the use of variance.
Action _UCTQTable::GetUCTAction(Size state_hash, Size& hash_a, bool explore) {
	// If state not in the tree the planner has to choose a huristic action, and not call this function.
	assert(inTree(state_hash));

	CStateInfoHash& state_info = _qdata[state_hash]; // we are sure here that state exists in the map.
	double logN = log(double(state_info->N));
	multimap<double,Size> pq; // pq of pairs <Vi, action's id in the list>
	for (Size t=0; t<_num_actions; t++){
		if (!inTree(state_hash,t)) {
			if (explore) {
				pq.insert(pair<double,Size>(INT_MAX, t));	// not attempted actions have very high valus.
			}
		}else{
			CActionInfoHash& action_info = state_info->_actions[t]; // for sure action t is in the tree
			if (action_info->Ni>0) {
				double Vi;
				Vi = double(action_info->Vi) / (double)action_info->Ni;
				if (explore) { // we do not add interval to during exploitation for the final action choice.
					double conf_interv = (2 * _rmax) * sqrt(logN / double(action_info->Ni)); // C=2*Rmax
					Vi += conf_interv;
				}
				pq.insert(pair<double,Size>(Vi,t));
			}else{
				if (explore) {
					pq.insert(pair<double,Size>(INT_MAX,t));
				}
			}
		}
	}
	if (LOGUCT > 0) {
		cout << "making decision:" << endl;
		for (multimap<double,Size>::const_iterator citer=pq.begin(); citer!=pq.end(); citer++) {
			cout << (*citer).first << " " << (*citer).second << Action(_domain, (*citer).second) << endl;
		}
		cout << endl;
	}
	multimap<double,Size>::iterator last = pq.end(); last--;
	double max = last->first;
	double n = pq.count(max); // how many elements in pq is equal to the max
	if (n==1) { // only one max
		hash_a = last->second;
		return Action(_domain, hash_a);
	}
	multimap<double,Size>::iterator itlow	=	pq.lower_bound(max);
	//multimap<int,int>::iterator itup	=	pq.upper_bound (max);
	int i = int( double(rand()) / (double(RAND_MAX) + 1.0) * n );
	for (int t=0; t<n; t++, itlow++){
		if (t==i){
			hash_a = itlow->second;
			break;
		}
	}
	return Action(_domain, hash_a);
}

void _UCTQTable::DoBackups(_CTraceList& trace, double cumulative_reward, bool full_tree) {
	// (*)
	if (_reward_type == 1) { // compute MC reward from each state
		for( int i = int(trace.size() - 2); i >= 0; i-- ) {
			trace[i]->reward += trace[i+1]->reward;
		}
	}

	// (*)
	if (full_tree) { // here the entire trace is in the Q-table
		for( int i = 0; i < int(trace.size()); i++ ) { /// The goal state is not in the trace.
			CActionInfoHash &qsa = _qdata[trace[i]->hash_s]->_actions[trace[i]->hash_a];
			if (_reward_type == 0) {
				qsa->Vi += cumulative_reward;	// final game reward
			} else {
				qsa->Vi += trace[i]->reward;	// cost to go reward
			}
		}
	} else { // extending the tree by one state form the trace.
		for( int i = 0; i < int(trace.size()); i++ ) { /// The goal state is not in the trace.
			if ( inTree(trace[i]->hash_s, trace[i]->hash_a) ) {
				CActionInfoHash &qsa = _qdata[trace[i]->hash_s]->_actions[trace[i]->hash_a];
				if (_reward_type == 0) {
					qsa->Vi += cumulative_reward;	// final game reward
				} else {
					qsa->Vi += trace[i]->reward;	// cost to go reward
				}
			} else { // add one and break, i.e., do not go further down
				Visit(trace[i]->hash_s, trace[i]->hash_a);
				CActionInfoHash &action_info = _qdata[trace[i]->hash_s]->_actions[trace[i]->hash_a];
				if (_reward_type == 0) {
					action_info->Vi = cumulative_reward;
				} else {
					action_info->Vi = trace[i]->reward;
				}
				return;
			}
		}
	}
}

// -----------------------------------------------------------------------------------

} // crl
