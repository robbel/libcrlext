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

#include <sys/time.h>

#include "crl/uctdata.hpp"
#include "crl/crl.hpp"
#include "crl/fdomain.hpp"
#include "crl/hdomain.hpp"

namespace crl {

// -------------------------------------------------------------------------------
// The UCT planner

/// The interface for UCT-based planners.
class _IUCTPlanner : public _Planner {
public:
	_IUCTPlanner(_IUCTQTable* qtable, Domain domain, MDP mdp, Reward gamma):
		_qtable(qtable), _domain(domain), _mdp(mdp), _gamma(gamma), _run_limit(0), _time_limit(0) {
		_clear_tree = false;
		_maxSteps = 0;
		_fullTree = true;
	}
	virtual ~_IUCTPlanner(){}

	void setRunLimit(Size run_limit) {_run_limit=run_limit;}
	void setTimeLimit(time_t time_limit) {_time_limit=time_limit;}
	void setConfidenceCoeff(float confidence_coeff){}
	/// from _Planner
	virtual Action getAction(const State& s);

protected:
	/// UCT Q-table
	IUCTQTable _qtable;
	/// The domain describes what valid states and actions are.
	Domain _domain;
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
	/// maximum number of steps for single trajectory (0, means unlimited trajectory until the goal is reached)
	int _maxSteps;
	/// If true than the full tree is created for each state visited during sampling.
	bool _fullTree;

	/// Performs one, complete UCT simulation.
	virtual void runSimulation(const State& s);
	/// Removes all the entries from the tree.
	virtual void ClearTree() {
		_qtable->Clear();
	}
	/// @return an action heuristically
	Action GetHuristicAction() const {
		return Action( _domain, Size((double(rand())/(double(RAND_MAX)+double(1.0)))  *  double(_domain->getNumActions())) );
	}
};


/// The UCT planner: basic UCT planner, no use of variance.
/// Direct implementations of _IUCTPlanner are to create _IUCTPlanner with proper IUCTQTable implementation.
class _UCTPlanner : public _IUCTPlanner {
public:
	virtual ~_UCTPlanner(){}
protected:
	_UCTPlanner(Domain domain, MDP mdp, Reward gamma)
		: _IUCTPlanner(new _UCTQTable(domain), domain, mdp, gamma){}
};
typedef boost::shared_ptr<_UCTPlanner> UCTPlanner;


/// _FlatUCTPlanner
class _FlatUCTPlanner : public _UCTPlanner {
public:
	_FlatUCTPlanner(Domain domain, MDP mdp, Reward gamma)
		: _UCTPlanner(domain, mdp, gamma){}
};

} // crl

#endif /*UCT_HPP_*/
