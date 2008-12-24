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
#include <sys/time.h>

#include "crl/crl.hpp"
#include "crl/vi.hpp"
#include "crl/fdomain.hpp"
#include "crl/hdomain.hpp"


namespace uct {
using namespace crl;

// --------------------------------------------------------------------------------------------
// Data structures for the Q-table (tree).

/// TODO: _UCTQTable should inherit form HASH_MAP<hash_key_type, CStateMBInfHash, KeyTypeHash>
/// TODO: copy to this file classes to store UCT related infomration

/// Implementation of the Q-table in UCT planning.
/// TODO: Keeps track of the best action and q-value for each state.
/// The point to have this class is that it will contain also UCT related infomration: counters, variance, etc.
class _UCTQTable : public _QTable, _StateActionTable<Reward> {
protected:
public:
	_UCTQTable(const Domain& domain);
	_UCTQTable(const Domain& domain, Reward initial);

	virtual Reward getQ(const State& s, const Action& a) {
		return _StateActionTable<Reward>::getValue(s, a);
	}
/*	virtual void setQ(const State& s, const Action& a, Reward r);
	virtual Reward getV(const State& s) {
		return _best_qs[s.getIndex()];
	}
	virtual Action getBestAction(const State& fs) {
		if (!_best_actions[fs.getIndex()])
			return _best_actions[fs.getIndex()] = Action(_domain);
		return _best_actions[fs.getIndex()];
	}*/
};
typedef boost::shared_ptr<_UCTQTable> UCTQTable;

}

namespace crl {

// -------------------------------------------------------------------------------
// The UCT planner

/**
 * The UCT planner.
 */
class _UCTPlanner : public _Planner {
protected:
	/**
	 * The domain describes what valid states and actions are.
	 */
	Domain _domain;
	/**
	 * The MDP to plan with
	 */
	MDP _mdp;
	/**
	 * The q-table to store values in
	 */
	QTable _qtable;
	/**
	 * A table to keep track of how many times a state has gone through a roll-out
	 */
	SCountTable _s_counts;

	/**
	 * discount rate
	 */
	Reward _gamma;
	/**
	 * unused at the moment
	 */
	Reward _epsilon;

	/**
	 * Max number of times a state can be in a roll-out
	 */
	Index _m;

	/**
	 * Max number of roll-outs per call to getAction
	 */
	Size _run_limit;
	/**
	 * Max number milliseconds per call to getAction
	 */
	time_t _time_limit;

	/**
	 * Likelihood that a roll-out explores a random action
	 */
	Probability _explore_epsilon;
	/**
	 * Max depth for a roll-out
	 */
	Size _max_depth;

	/**
	 * used for its ability to perform Bellman backups
	 */
	VIPlanner _vi_planner;

	/**
	 * Perform a simulation/roll-out starting at s. Returns the biggest
	 * Bellman residual.
	 */
	Reward runSimulation(const State& s, Size depth=0);

	_UCTPlanner(Domain domain, MDP mdp, QTable qtable, SCountTable s_counts,
	             Reward gamma, Reward epsilon, Index m,
	             Probability explore_epsilon, Size max_depth);
public:
	virtual ~_UCTPlanner() { }

	void setRunLimit(Size run_limit) {_run_limit=run_limit;}
	void setTimeLimit(time_t time_limit) {_time_limit=time_limit;}
	void setConfidenceCoeff(float confidence_coeff){}

	/**
	 * from _Planner
	 */
	virtual Action getAction(const State& s);
};
typedef boost::shared_ptr<_UCTPlanner> UCTPlanner;

/**
 * This is a subclass that provides the necessary data structures. This
 * version of RTDPPlanner provides a flat q-table and state count table.
 */
class _FlatUCTPlanner : public _UCTPlanner {
public:
	_FlatUCTPlanner(Domain domain, MDP mdp,
		             Reward gamma, Reward epsilon, Index m,
		             Probability explore_epsilon, Size max_depth);
};

}


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
