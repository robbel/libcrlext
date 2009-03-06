/*
 * uctdata.cpp
 *
 *  Created on: 25-Dec-2008
 *      Author: Marek Grzes, University of York
 */

#include "crl/uctdata.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

#define LOGUCT 0

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

Action _IUCTQTable::GetUCTAction(Size state_hash, Size& hash_a, bool explore) {
	// If state not in the tree the planner has to choose a huristic action, and not call this function.
	assert(inTree(state_hash));

	CStateInfoHash& state_info = _qdata[state_hash]; // we are sure here that state exists in the map.
	double logN = log(double(state_info->N));
	multimap<double,Size> pq; // pq of pairs <Vi, action's id in the list>
	for (Size t=0; t<_num_actions; t++){
		if (!inTree(state_hash,t)) {
			if (explore) {
				pq.insert(pair<double,Size>(std::numeric_limits<int>::max(), t));	// not attempted actions have very high valus.
			}
		}else{
			CActionInfoHash& action_info = state_info->_actions[t]; // for sure action t is in the tree
			if (action_info->Ni>0) {
				Reward Vi = GetUCTValue(action_info, logN, explore);
				pq.insert(pair<double,Size>(Vi, t));
			}else{
				if (explore) {
					pq.insert(pair<double,Size>(std::numeric_limits<int>::max(),t));
				}
			}
		}
	}
	if (LOGUCT > 0) {
		cerr << "making decision: in state " << State(_domain, state_hash) << endl;
		for (multimap<double,Size>::const_iterator citer=pq.begin(); citer!=pq.end(); citer++) {
			cerr << "v=" << (*citer).first << " action_hash=" << (*citer).second << " action=" << Action(_domain, (*citer).second) << endl;
		}
	}
	multimap<double,Size>::iterator last = pq.end(); last--;
	double max = last->first;
	double n = pq.count(max); // how many elements in pq is equal to the max
	if (n==1) { // only one max
		hash_a = last->second;
		if (LOGUCT > 0) {
			cerr << "selected action = " << Action(_domain, hash_a) << endl << endl;
		}
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
	if (LOGUCT > 0) {
		cerr << "selected action = " << Action(_domain, hash_a) << endl << endl;
	}
	return Action(_domain, hash_a);
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
Reward _UCTQTable::GetUCTValue(CActionInfoHash& action_info, double logN, bool explore) {
	assert(action_info->Ni > 0 && "call this methods only when you can compute a valid value");
	double Vi;
	Vi = double(action_info->Vi) / (double)action_info->Ni;
	if (explore) { // we do not add interval to during exploitation for the final action choice.
		double conf_interv = (2 * _C) * sqrt(logN / double(action_info->Ni)); // C=2*Rmax
		Vi += conf_interv;
	}
	return Vi;
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
// _MBUCTQTable Q-table

Reward 	_MBUCTQTable::getQ (const State &s, const Action &a) {
	_CStateInfoMBHash* temp = static_cast<_CStateInfoMBHash*>(_qdata[s.getIndex()].get());
	return temp->_actions[a.getIndex()]->Vi;
}

void 	_MBUCTQTable::setQ (const State &s, const Action &a, Reward r) {
	_CStateInfoMBHash* temp = static_cast<_CStateInfoMBHash*>(_qdata[s.getIndex()].get());
	if (temp->V < r) {
		temp->V = r; // set the value of this state to the max value of action.
	}
	temp->_actions[a.getIndex()]->Vi = r;
}

Action 	_MBUCTQTable::getBestAction (const State &s) {
	_CStateInfoMBHash* state_info = static_cast<_CStateInfoMBHash*>(_qdata[s.getIndex()].get());

	Size best_action_hash;
	Reward max;
	bool first = true;
	for( HASH_MAP_ACTIONS_INFO::const_iterator iter = state_info->_actions.begin(); iter != state_info->_actions.end(); iter++) {
		const CActionInfoHash& action_info = iter->second;
		if (first) {
			best_action_hash = iter->first;
			max =  action_info->Vi;
			first = false;
		} else {
			Reward current =  action_info->Vi;
			if (current > max) {
				max = current;
				best_action_hash = iter->first;
			}
		}
	}
	return Action(_domain, best_action_hash);
}

Reward 	_MBUCTQTable::getV (const State &s) {
	_CStateInfoMBHash* temp = static_cast<_CStateInfoMBHash*>(_qdata[s.getIndex()].get());
	return temp->V;
}

// Standard UCT without the use of variance.
Reward _MBUCTQTable::GetUCTValue(CActionInfoHash& action_info, double logN, bool explore) {
	assert(action_info->Ni > 0 && "call this methods only when you can compute a valid value");
	double Vi;
	// (*) the Q(s,a) value of this action
	Vi = action_info->Vi;
	// (*) exploration bonus
	if (explore) { // we do not add interval to during exploitation for the final action choice.
		double conf_interv = (2 * _C) * sqrt(logN / double(action_info->Ni)); // C=2*Rmax
		Vi += conf_interv;
	}
	return Vi;
}

// TODO: implement this for MBUCT
void _MBUCTQTable::DoBackups(_CTraceList& trace, double cumulative_reward, bool full_tree) {
/*
	// (*)
	if (full_tree) { // here the entire trace is in the Q-table
		for( int i = int(trace.size() - 1); i >= 0; i-- ) { /// The goal state is not in the trace.
			m_Qtable[trace[i]->hash_s].backup(m_Qtable); // VI
		}
	} else { // extending the tree by one state form the trace.
		double back_reward = 0; // need this because the tail of the trace is not in the tree
		bool prev_in_tree = false;

		if ( inTree(trace.back()->hash_s, trace.back()->hash_a) ) {
			// Update of the last but one state. Here V=0 will be used in the goal state.
			m_Qtable[trace.back()->hash_s].backup(m_Qtable); // VI
			prev_in_tree = true;
		}
		back_reward = trace.back()->reward;
		if ( inTree(trace.back()->hash_s) ) { // Other action for this state can be in the tree.
			CStateInfoMBHash& vs = _qdata[trace.back()->hash_s];
			if (back_reward > vs.V) {
				vs.V = back_reward; // in this trajectory this state got higher value via not stored action;
			}else{
				back_reward = vs.V; // stored action had higher value;
			}
		}

		for( int i = int(trace.size() - 2); i >= 0; i-- ) {
			back_reward += trace[i]->reward;
			if ( inTree(trace[i]->hash_s, trace[i]->hash_a) ) {
				if (prev_in_tree == true) { // just update of this state
					// (*) update current
					double v = m_Qtable[trace[i]->hash_s].backup(m_Qtable); // VI
					back_reward = std::max(back_reward, v);
				} else {
					// (*) add previous
					Visit(trace[i+1]->hash_s, trace[i+1]->hash_a);
					// The next state of the added state becomes a leaf state with a value from the trace.
					double leaf_value = back_reward - trace[i]->reward - trace[i+1]->reward; // dwa kolejne dodania przed lisciem odejmujemy
					AddLeaf2Tree(trace[i+1].hash_next_s, leaf_value);
					// After adding the leaf we can compute normal backup of state [i+1]
					double v = m_Qtable[trace[i+1]->hash_s].backup(m_Qtable); // VI
					if ( v > back_reward - trace[i]->reward ) {
						back_reward = v + trace[i]->reward; // other action might have higher value in the added state
					}
					// (*) update current
					v = m_Qtable[trace[i]->hash_s].backup(m_Qtable); // VI
					back_reward = std::max(back_reward, v);
				}
				prev_in_tree = true;
			} else {
				if ( i == 0 ) { // to add the first (current) state to the tree
					// (*) add current state
					Visit(trace[i]->hash_s, trace[i]->hash_a);
					// The next state of the added state becomes a leaf state with a value from the trace.
					double leaf_value = back_reward - trace[i]->reward;
					AddLeaf2Tree(trace[i].hash_next_s, leaf_value);
					// After adding the leaf we can compute normal backup of state [i+1]
					m_Qtable[trace[i]->hash_s].backup(m_Qtable); // VI
				} else {
					if ( inTree(trace[i]->hash_s) ) { // Other action for this state can be in the tree.
						CStateInfoMBHash& vs = _qdata[trace[i]->hash_s];
						if (back_reward > vs.V) {
							vs.V = back_reward; // in this trajectory this state got higher value via not stored action;
						}else{
							back_reward = vs.V;
						}
					}
				}
				prev_in_tree = false;
			}
		}
	}
*/
}

// -----------------------------------------------------------------------------------

} // crl
