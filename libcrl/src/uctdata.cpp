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

bool _IUCTQTable::inTree(hash_key_type hash_s, hash_key_type hash_a) const {
	HASH_MAP_STATE_INFO::const_iterator state = _qdata.find(hash_s);
	if(state != _qdata.end()){
		HASH_MAP_ACTIONS_INFO::const_iterator action = state->second->_actions.find(hash_a);
		if(action != state->second->_actions.end()){
			return true;
		}
	}
	return false;
}

bool _IUCTQTable::inTree(hash_key_type hash_s) const {
	HASH_MAP_STATE_INFO::const_iterator state = _qdata.find(hash_s);
	if(state != _qdata.end()){
		return true;
	}
	return false;
}

template <class S, class _S, class A, class _A>
void _IUCTQTable::Visit(hash_key_type hash_s, hash_key_type hash_a, const State& s, const Action& a){
	if( !inTree(hash_s)) {
		S temp(new _S());
		temp->N = 1;
		temp->_state = s;
		_qdata[hash_s] = temp;
	}else{
		_qdata[hash_s]->N++;
	}
	if ( !inTree(hash_s, hash_a)) {
		A temp(new _A());
		temp->Ni=1;
		temp->_action = a;
		_qdata[hash_s]->_actions[hash_a] = temp;
	}else{
		_qdata[hash_s]->_actions[hash_a]->Ni++;
	}
}

// -----------------------------------------------------------------------------------
// _UCTQTable Q-table

Action 	_UCTQTable::getBestAction (const State &s) {
	CStateInfoHash& state_info = _qdata[s.getIndex()];
	CActionInfoHash best_action;
	Reward max;
	bool first = true;
	for( HASH_MAP_ACTIONS_INFO::const_iterator iter = state_info->_actions.begin(); iter != state_info->_actions.end(); iter++) {
		const CActionInfoHash& action_info = iter->second;
		if (first) {
			best_action = action_info;
			max =  Reward( action_info->Vi / double(action_info->Ni) );
			first = false;
		} else {
			Reward current =  Reward( action_info->Vi / double(action_info->Ni) );
			if (current > max) {
				max = current;
				best_action = action_info;
			}
		}
	}
	return best_action->_action;
}

// Standard UCT without the use of variance.
Action _UCTQTable::GetUCTAction(
hash_key_type state_hash, vector<hash_key_type>& hash_list,
hash_key_type& hash_a, bool explore
) {
	if (!inTree(state_hash)) { // State not in the tree, choose random action.
		size_t i = size_t(rand()/(RAND_MAX + 1.0)*hash_list.size());
		hash_a = hash_list[i];
		return _qdata[state_hash]->_actions[hash_a]->_action;
	}
	CStateInfoHash& state_info = _qdata[state_hash]; // we are sure here that state exists in the map.
	double logN = log(double(state_info->N));
	size_t an = hash_list.size();
	multimap<double,hash_key_type> pq; // pq of pairs <Vi, action's id in the list>
	for (size_t t=0; t<an; t++){
		if (!inTree(hash_list[t])) {
			if (explore) {
				pq.insert(pair<double,hash_key_type>(INT_MAX, hash_list[t]));	// not attempted actions have very high valus.
			}
		}else{
			CActionInfoHash& action_info = state_info->_actions[hash_list[t]]; // for sure action t is in the tree
			if (action_info->Ni>0) {
				double Vi;
				Vi = double(action_info->Vi) / (double)action_info->Ni;
				if (explore) { // we do not add interval to during exploitation for the final action choice.
					double conf_interv = (2 * _rmax) * sqrt(logN / double(action_info->Ni)); // C=2*Rmax
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
	if (LOGUCT > 0) {
		cout << "making decision:" << endl;
		for (multimap<double,hash_key_type>::const_iterator citer=pq.begin(); citer!=pq.end(); citer++) {
			cout << (*citer).first << " " << (*citer).second << endl;
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
		return _qdata[state_hash]->_actions[a_index]->_action;
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
	return _qdata[state_hash]->_actions[a_index]->_action;
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
				Visit(trace[i]->hash_s, trace[i]->hash_a, trace[i]->_state, trace[i]->_action);
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
