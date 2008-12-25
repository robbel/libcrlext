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

#ifndef UCT_HPP_
#define UCT_HPP_

#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <cassert>
#include <sys/time.h>
#if defined(__linux__) || defined(__CYGWIN__)
#include <ext/hash_map>
#define HASH_MAP hash_map
#endif

#include "crl/crl.hpp"
#include "crl/fdomain.hpp"
#include "crl/hdomain.hpp"

namespace crl {
#if defined(__linux__) || defined(__CYGWIN__)
using namespace __gnu_cxx;
#endif

typedef  uint64_t hash_key_type;	// in this way we can easily change the type of the key.


// --------------------------------------------------------------------------------------------
// Data structures for the Q-table (tree).


struct KeyTypeHash {
	inline size_t operator()(const hash_key_type& s) const {
		return size_t(s);
	}
};

/// The class to store information related to a specific action in a state. For the use with the
/// simplest UCT.
struct _CActionInfoHash{
public:
	_CActionInfoHash(double Wi_=0, int Ni_=0){
		Ni = Ni_;
		Wi = Wi_;
	}
	~_CActionInfoHash(void){}
	_CActionInfoHash& operator=(const _CActionInfoHash& entry){
		this->Ni = entry.Ni;
		this->Wi = entry.Wi;
		return *this;
	}
	Reward getValue() const {
		assert(Ni != 0 && "this should not happen");
		return Wi/Ni;
	}
	/// The number of times an action has been attempted.
	int Ni;
	/// The sum of scores.
	double Wi;
};

/// The value in the hash table related to one specific state. It contains a hash table with actions.
class _CStateInfoHash: public HASH_MAP<hash_key_type, _CActionInfoHash, KeyTypeHash>{
public:
	_CStateInfoHash(int N_=0){
		N = N_;
	}
	_CStateInfoHash& operator=(const _CStateInfoHash& entry){
		this->N = entry.N;
		// We have to call this operator also on the base class (TODO: can we do it in better way?).
		HASH_MAP<hash_key_type, _CActionInfoHash, KeyTypeHash> *parent = static_cast<HASH_MAP<hash_key_type, _CActionInfoHash, KeyTypeHash>*>(this);
		parent->operator = (entry);
		return *this;
	}
	/// @return true if action hash_a is in the tree in the state this
	bool inTree(hash_key_type hash_a) const{
		if (this->find(hash_a) != this->end()) {
			return true;
		}
		return false;
	}
	// The number of times the state has been visited.
	int N;
	// The value function.
};

/// Implementation of the Q-table in UCT planning.
/// The point to have this class is that it will contain also UCT related information: counters, variance, etc.
class _UCTQTable : public /*_QTable, _StateActionTable<Reward>,*/ HASH_MAP<hash_key_type, _CStateInfoHash, KeyTypeHash> {
protected:
public:
	_UCTQTable(const Domain& domain) {}
	_UCTQTable(const Domain& domain, Reward initial) {}

	/// Add state action pair (s,a) to the three (to the hash map).
	virtual void Add2Tree(hash_key_type hash_s, hash_key_type hash_a);

	/// @return the hash code of the best action (exploitation only, no bonuses are added)
	virtual hash_key_type getBestActionHash(hash_key_type& hash_s);

	/// @return true if pair (s,a) is in the tree (in the hash map).
	virtual bool inTree(hash_key_type hash_s, hash_key_type hash_a) const;

	/// @return true if state s is in the tree
	virtual bool inTree(hash_key_type hash_s) const;

};
typedef boost::shared_ptr<_UCTQTable> UCTQTable;

/* ====================================================================== */
/* Trace data structure for storing the trace */

/// What we store along the trace.
struct CTraceData{
	CTraceData(hash_key_type hash_s_, hash_key_type hash_a_, Reward reward_): hash_s(hash_s_), hash_a(hash_a_), reward(reward_){}
	CTraceData& operator=(const CTraceData& entry){
		hash_s = entry.hash_s;
		hash_a = entry.hash_a;
		reward = entry.reward;
		return *this;
	}
	/// hash of this state
	hash_key_type hash_s;
	/// hash of the action
	hash_key_type hash_a;
	/// reward obtained in this transition
	Reward reward;
};
struct CTraceList: public vector<CTraceData>{};


// -------------------------------------------------------------------------------
// The UCT planner

/// typedef vector<Action> ActionList;
struct ActionList: public vector<Action>{
	/// Intialise this object with ActionIterator data.
	ActionList(ActionIterator& actions){
		actions->reset();
		while (actions->hasNext()) {
			this->push_back(actions->next());
		}
	}
};

/// The UCT planner.
class _UCTPlanner : public _Planner {
public:
	virtual ~_UCTPlanner(){}

	void setRunLimit(Size run_limit) {_run_limit=run_limit;}
	void setTimeLimit(time_t time_limit) {_time_limit=time_limit;}
	void setConfidenceCoeff(float confidence_coeff){}

	/// from _Planner
	virtual Action getAction(const State& s);

protected:
	_UCTPlanner(Domain domain, MDP mdp, Reward gamma);

	/// The domain describes what valid states and actions are.
	Domain _domain;

	/// UCT Q-table
	_UCTQTable _qtable;

	/// The MDP to plan with
	MDP _mdp;

	/// discount rate
	Reward _gamma;

	/// Max number of roll-outs per call to getAction
	Size _run_limit;

	/// Max number milliseconds per call to getAction
	time_t _time_limit;

	/// when true, there is no reuse of the qtable.
	bool _clear_tree;

	/// If true than the full tree is created for each state visited during sampling.
	bool _fullTree;

	/// 0 - the final game reward, 1 - the sum of step rewards from a given state.
	int _reward_type;

	/// Max value of reward (to be used to find the scaling factor C for UCT exploration bonus.
	Reward _rmax;

	/// Performs one, complete UCT simulation.
	void runSimulation(const State& s);

	/// Removes all the entries from the tree.
	virtual void ClearTree() {
		_qtable.clear();
	}
	/// @return an action heuristically
	/// @return returns an action heuristically (in this implementation randomly selected).
	Action GetHuristicAction(ActionList& actions) const {
		return actions[size_t(  (double(rand())/(double(RAND_MAX)+double(1.0)))  *  double(actions.size())  )];
	}

	/// Update counter N and Ni.
	/// Parameters like V or Wi are not updated here (DoBackups does updates of those parameters).
	virtual void UpdateModel(hash_key_type hash_s, hash_key_type hash_a, hash_key_type hash_sprime, double reward);

	/// Update Wi for state in the trajectory.
	virtual void DoBackups(CTraceList& trace, double cumulative_reward);

	/// @return action
	/// @param state_hash - the hash code of the state for which we want to have an action.
	/// @param actions - the list of actions
	/// @param hash_list - hashes of actions in state state_hash.
	/// @param[out] hash_a - the hash code of the action which is returned by this function
	/// @param explore - if true do exploration, false exploitation
	virtual Action GetUCTAction(hash_key_type state_hash, ActionList& actions, vector<hash_key_type>& hash_list, hash_key_type& hash_a, bool explore=true);
};

typedef boost::shared_ptr<_UCTPlanner> UCTPlanner;

/**
 * This is a subclass that provides the necessary data structures. This
 * version of RTDPPlanner provides a flat q-table and state count table.
 */
class _FlatUCTPlanner : public _UCTPlanner {
public:
	_FlatUCTPlanner(Domain domain, MDP mdp, Reward gamma);
};

Size iterator_size(ActionIterator& actions);

} // crl


/*

#include <math.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "crl/crl.hpp"
#include "crl/vi.hpp"
#include "crl/ps.hpp"
#include "crl/fdomain.hpp"

namespace crl {

class _UCTPlanner : public _Planner {
protected:
	const MDP _mdp;
	float _confidence_coeff;
	float _gamma;
	time_t _time_limit;
	int _run_limit;
	float _learning_rate; //if 0, average
	StateSet _frequent_states;
	PSPlanner _ps_planner;

	_UCTPlanner(const MDP& mdp, float confidence_coeff, float gamma);

	virtual bool isTerminal(const State& s, Size depth) = 0;
	virtual int getVisits(const State& s, Size depth) = 0;
	virtual int getVisits(const State& s, const Action& a, Size depth) = 0;
	virtual void setQ(const State& s, const Action& a, Reward q, Size depth) = 0;
	virtual const Reward getQ(const State& s, const Action& a, Size depth) = 0;
	virtual void incrVisits(const State& s, const Action& a, Size depth) = 0;


	Reward getConfidence(const State& s, const Action& a, Size depth);
	Action selectAction(ActionIterator& aitr, const State& s, Size depth);
	virtual void avgQ(const State& s, const Action& a, Reward q, Size depth);

	Reward runSimulation(const State& initial_state, Size depth=0);
	virtual QTable getQTable(Size depth) = 0;

public:
	void setTimeLimit(time_t time_limit);
	void setRunLimit(int run_limit);
	void setConfidenceCoeff(float confidence_coeff);
	void setLearningRate(float learning_rate);
	QTable getQTable();
	virtual Action getAction(const State& s);
};
typedef boost::shared_ptr<_UCTPlanner> UCTPlanner;

class _FactoredUCTPlanner : public _UCTPlanner {
protected:
	const Domain _domain;
	std::vector<FQTable> _qtables;
	_FStateActionTable<std::vector<Size> > _sa_visits;
	_FStateTable<std::vector<Size> > _s_visits;

	virtual bool isTerminal(const State& s, Size depth);
	virtual int getVisits(const State& s, Size depth);
	virtual int getVisits(const State& s, const Action& a, Size depth);
	virtual void setQ(const State& s, const Action& a, Reward q, Size depth);
	virtual const Reward getQ(const State& s, const Action& a, Size depth);
	virtual void incrVisits(const State& s, const Action& a, Size depth);
	virtual QTable getQTable(Size depth);
public:
	_FactoredUCTPlanner(const Domain& domain, const MDP& mdp, float confidence_bias, float gamma);
};
typedef boost::shared_ptr<_FactoredUCTPlanner> MapUCTPlanner;

}
*/

#endif /*UCT_HPP_*/
