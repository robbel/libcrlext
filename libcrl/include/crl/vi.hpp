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
#include "crl/fdomain.hpp"
#include "crl/hdomain.hpp"
#include "crl/mdomain.hpp"

#ifdef NDEBUG
HIETEKHRSD
#endif

namespace crl {

class _VIPlanner : public _Planner {
protected:
	const MDP _mdp;
	QTable _qtable;
	Reward _epsilon;
	float _gamma;

	virtual Reward backupState(const State& s, ActionIterator& aitr);
	virtual Reward backupStateAction(const State& s, const Action& a);

	_VIPlanner(const MDP& mdp, Reward epsilon, float gamma);
public:
	_VIPlanner(const MDP& mdp, Reward epsilon, float gamma, QTable qtable);
	virtual int plan(StateIterator& sitr, ActionIterator& aitr);
	int plan();
	virtual Action getAction(const State& s);
	QTable getQTable();
};
typedef boost::shared_ptr<_VIPlanner> VIPlanner;
inline VIPlanner getVIPlanner(const Planner& planner) {
	return boost::shared_polymorphic_downcast<_VIPlanner>(planner);
}

class _FactoredVIPlanner : public _VIPlanner {
public:
	_FactoredVIPlanner(const Domain& domain, const MDP& mdp, Reward epsilon, float gamma)
	: _VIPlanner(mdp, epsilon, gamma) {
		_qtable = FQTable(new _FQTable(domain, 0));
	}
};
typedef boost::shared_ptr<_FactoredVIPlanner> FactoredVIPlanner;

class _HashedVIPlanner : public _VIPlanner {
public:
	_HashedVIPlanner(const Domain& domain, const MDP& mdp, Reward epsilon, float gamma)
	: _VIPlanner(mdp, epsilon, gamma) {
		_qtable = HQTable(new _HQTable(domain));
	}
};
typedef boost::shared_ptr<_HashedVIPlanner> HashedVIPlanner;

class _MappedVIPlanner : public _VIPlanner {
public:
	_MappedVIPlanner(const MDP& mdp, Reward epsilon, float gamma)
	: _VIPlanner(mdp, epsilon, gamma) {
		_qtable = MQTable(new _MQTable(0));
	}
};
typedef boost::shared_ptr<_MappedVIPlanner> MappedVIPlanner;

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
