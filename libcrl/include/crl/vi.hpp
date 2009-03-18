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

#ifndef VI_H_
#define VI_H_

#include <iostream>
#include "crl/crl.hpp"
#include "crl/flat_tables.hpp"
#include "crl/hash_tables.hpp"

namespace crl {

/**
 * A planner that does value iteration and provides
 * Bellman backup functionality
 */
class _VIPlanner : public _Planner {
protected:
	/**
	 * the dynamics model
	 */
	MDP _mdp;
	/**
	 * The q-table to store values in
	 */
	QTable _qtable;
	/**
	 * VI stops when all Bellman residuals drop below epsilon
	 */
	Reward _epsilon;
	/**
	 * discount factor
	 */
	float _gamma;

	_VIPlanner(MDP mdp, Reward epsilon, float gamma);
public:
	_VIPlanner(MDP mdp, Reward epsilon, float gamma, QTable qtable);
	/**
	 * Do VI on some subset of the state and action space
	 */
	virtual int plan(StateIterator sitr, ActionIterator aitr);
	/**
	 * Do VI on _mdp->S(), _mdp->A()
	 */
	int plan();

	/**
	 * perform a Bellman backup on some state, for all actions provided
	 */
	virtual Reward backupState(const State& s, ActionIterator& aitr);
	/**
	 * Backup the value for a single state/action
	 */
	virtual Reward backupStateAction(const State& s, const Action& a);
	/**
	 * Find the new value for a state/action
	 */
	virtual Reward evaluateStateAction(const State& s, const Action& a);
	/**
	 * Get the action that maximizes Q(s,a)
	 */
	virtual Action getAction(const State& s);
	QTable getQTable();
};
typedef boost::shared_ptr<_VIPlanner> VIPlanner;
inline VIPlanner getVIPlanner(const Planner& planner) {
	return boost::shared_polymorphic_downcast<_VIPlanner>(planner);
}

/**
 * VI planner using a flat table
 */
class _FactoredVIPlanner : public _VIPlanner {
public:
	_FactoredVIPlanner(const Domain domain, MDP mdp, Reward epsilon, float gamma)
	: _VIPlanner(mdp, epsilon, gamma) {
		_qtable = FQTable(new _FQTable(domain));
	}
};
typedef boost::shared_ptr<_FactoredVIPlanner> FactoredVIPlanner;

/**
 * VI planner using a hash map
 */
class _HashedVIPlanner : public _VIPlanner {
public:
	_HashedVIPlanner(const Domain domain, MDP mdp, Reward epsilon, float gamma)
	: _VIPlanner(mdp, epsilon, gamma) {
		_qtable = HQTable(new _HQTable(domain));
	}
};
typedef boost::shared_ptr<_HashedVIPlanner> HashedVIPlanner;

/**
 * An agent that uses any learner, and replans w/ VI whenever the model changes.
 */
class _VIPlannerAgent : public _Agent {
public:
	_VIPlannerAgent(VIPlanner planner, Learner learner);
	virtual ~_VIPlannerAgent() { }
	virtual bool observe(const Observation& o);
};
typedef boost::shared_ptr<_VIPlannerAgent> VIPlannerAgent;
}

#endif /*VI_H_*/
