/*
 * uctdata.hpp
 *
 *  Created on: 25-Dec-2008
 *      Author: Marek Grzes, University of York
 *
 * To add a new UCT-based planner you need: 1) implement _IUCTQTable (data structures are flexible, so should allow
 * for different algorithms), 2) implement _IUCTPlanner, this step is now only to create the Q-table implemented
 * in 1) and pass it to _IUCTPlanner. Now, all algorithm specific code is in particular implementation of _IUCTQTable.
 */

#ifndef UCTDATA_HPP_
#define UCTDATA_HPP_

#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <cassert>
#if defined(__MACH__) ||  defined(__linux__) || defined(__CYGWIN__)
#include <ext/hash_map>
#define HASH_MAP hash_map
#endif

#include "crl/crl.hpp"

#if defined(__MACH__) ||  defined(__linux__) || defined(__CYGWIN__)
using namespace __gnu_cxx;
#endif

namespace crl{

// ------------------------------------------------------------------------------------
// Trace data structure for storing the trace

/// What we store along the trace.
struct _CTraceData{
	_CTraceData(Size hash_s_, Size hash_a_, Size hash_next_s_, double reward_): hash_s(hash_s_), hash_a(hash_a_), hash_next_s(hash_next_s_), reward(reward_){}
	/// hash of this state
	Size hash_s;
	/// hash of the action
	Size hash_a;
	// hash of the next state
	Size hash_next_s;
	/// reward in this transition
	double reward;
};
typedef boost::shared_ptr<_CTraceData> CTraceData;

/// The trace list is now a vector of shared pointers.
struct _CTraceList: public vector<CTraceData>{};

// ------------------------------------------------------------------------------------
// Data structures for the Q-table

/// The hash function for keys in the Q-table
struct KeyTypeHash {
	inline size_t operator()(const Size& s) const {
		return size_t(s);
	}
};

/// The class to store UCT related information for a specific action in a state.
/// To be used with standard UCT and MB-UCT with standard exploration.
class _CActionInfoHash{
public:
	_CActionInfoHash(double Vi_=0, int Ni_=0){
		Ni = Ni_;
		Vi = Vi_;
	}
	virtual ~_CActionInfoHash(void){}
	/// The number of times this action has been attempted.
	int Ni;
	/// The sum of scores in standard UCT or Value of this action in model-based planning.
	double Vi;
};
typedef boost::shared_ptr<_CActionInfoHash> CActionInfoHash;

/// To be used with UCT which needs variance of vi.
class _CActionInfoVHash: public _CActionInfoHash{
public:
	_CActionInfoVHash(double Vi_=0, int Ni_=0, double Meani_=0, double Sumsqi_=0, int NMeani_=0):_CActionInfoHash(Vi_, Ni_){
		Meani = Meani_;
		Sumsqi = Sumsqi_;
		NMeani = NMeani_;
	}
	~_CActionInfoVHash(void){}
	/// @return variance of Vi
	double get_variance(){
		if(NMeani>1) return Sumsqi / (NMeani-1.0);
		return 0.0;
	}
	/// @param[in] x next sample
	void incremental_variance(double x){
		NMeani++;
		double dev = x - Meani;
		Meani = Meani + dev/double(NMeani);
		Sumsqi = Sumsqi + dev*(x - Meani);
	}
protected:
	/// incremental estimate of the mean
	double Meani;
	/// incremental estimate of the sum of squares
	double Sumsqi;
	/// N - for calculation of Mean and Variance.
	int NMeani;
};
typedef boost::shared_ptr<_CActionInfoVHash> CActionInfoVHash;
typedef HASH_MAP<Size, CActionInfoHash, KeyTypeHash> HASH_MAP_ACTIONS_INFO;

/// The value in the hash table related to one specific state. It contains a hash table with actions.
class _CStateInfoHash{
public:
	_CStateInfoHash(int N_=0){
		N = N_;
	}
	/// Values are smart pointers to the basic CActionInfoHash class.
	HASH_MAP_ACTIONS_INFO _actions;
	/// The number of times the state has been visited.
	int N;
};
typedef boost::shared_ptr<_CStateInfoHash> CStateInfoHash;

/// Extension which adds state value to be used with MB-UCT
class _CStateInfoMBHash: public _CStateInfoHash{
public:
	_CStateInfoMBHash(int N_=0, double V_=0):_CStateInfoHash(N_){
		V = V_;
	}
	/// The value of this state
	double V;
};
typedef boost::shared_ptr<_CStateInfoMBHash> CStateInfoMBHash;
typedef HASH_MAP<Size, CStateInfoHash, KeyTypeHash> HASH_MAP_STATE_INFO;

// -----------------------------------------------------------------------------------
// UCT Q-tables

/// The basic interface for Q-tables for UCT algorithms.
class _IUCTQTable : public _QTable {
protected:
	HASH_MAP<Size, CStateInfoHash, KeyTypeHash> _qdata;
	/// Max value of reward (to be used to find the scaling factor C for UCT exploration bonus.
	Reward _rmax;
	/// 0 - the final game reward, 1 - the sum of step rewards from a given state.
	int _reward_type;
	/// Shrared pointer to the domain (for two direction action <-> hash and state <-> hash mappings).
	Domain _domain;
	/// The number of acgtions (the same for all states !!!).
	Size _num_actions;

public:
	_IUCTQTable(const Domain& domain, int reward_type){
		_domain = domain;
		_rmax = domain->getRewardRange().getMax();
		_num_actions = domain->getNumActions();
		_reward_type = reward_type;
	}
	_IUCTQTable(const Domain& domain, int reward_type, Reward initial) {
		_domain = domain;
		_rmax = domain->getRewardRange().getMax();
		_num_actions = domain->getNumActions();
		_reward_type = reward_type;
	}
	virtual ~_IUCTQTable(){}

	/// Remove all the data in the Q-table hash map.
	virtual void Clear() {
		_qdata.clear();
	}

	/// Increase counters and add to the tree if state is not there.
	template <class S, class _S, class A, class _A>
	void Visit(Size hash_s, Size hash_a);
	/// Implementation of this methods should call propertly parametrised template above.
	virtual void Visit(Size hash_s, Size hash_a) = 0;

	/// @return true if pair (s,a) is in the tree (in the hash map).
	virtual bool inTree(Size hash_s, Size hash_a) const;
	virtual bool inTree(const State& s, const Action& a) const {
		return inTree(Size(s.getIndex()), Size(a.getIndex()));
	}

	/// @return true if state s is in the tree
	virtual bool inTree(Size hash_s) const;
	virtual bool inTree(const State& s) const {
		return inTree(Size(s.getIndex()));
	}

	/// @return action
	/// @param state_hash - the hash code of the state for which we want to have an action.
	/// @param hash_list - hashes of actions in state state_hash.
	/// @param[out] hash_a - the hash code of the action which is returned by this function
	/// @param explore - if true do exploration
	virtual Action GetUCTAction(Size state_hash, Size& hash_a, bool explore=true) = 0;

	/// Update learned parameters for states which are in the trace.
	/// @param cumulative_reward - a final score at the end of the game.
	virtual void DoBackups(_CTraceList& trace, double cumulative_reward, bool full_tree) = 0;

	// Methods from _QTable
	virtual Reward 	getQ (const State &s, const Action &a) = 0;
	virtual void 	setQ (const State &s, const Action &a, Reward r) = 0;
	virtual Action 	getBestAction (const State &s) = 0;
	virtual Reward 	getV (const State &s) = 0;
};
typedef boost::shared_ptr<_IUCTQTable> IUCTQTable;

/// The Q-table for standard UCT.
class _UCTQTable : public _IUCTQTable {
public:
	_UCTQTable(const Domain& domain, int reward_type): _IUCTQTable(domain, reward_type){}

	virtual void Visit(Size hash_s, Size hash_a){
		_IUCTQTable::Visit<CStateInfoHash, _CStateInfoHash, CActionInfoHash, _CActionInfoHash>(hash_s, hash_a);
	}

	virtual Action GetUCTAction(Size state_hash, Size& hash_a, bool explore=true);

	virtual void DoBackups(_CTraceList& trace, double cumulative_reward, bool full_tree);

	virtual Reward 	getQ (const State &s, const Action &a) { return 0.0;}
	virtual void 	setQ (const State &s, const Action &a, Reward r) {}
	virtual Action 	getBestAction (const State &s);
	virtual Reward 	getV (const State &s) {return 0.0;}
};

// ------------------------------------------------------------------------------------

} // crl

#endif /* UCTDATA_HPP_ */
