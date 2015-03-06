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

#ifndef CRL_H_
#define CRL_H_

#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <boost/shared_ptr.hpp>
#include <cpputil.hpp>
#include "crl/util.hpp"
#include "crl/common.hpp"

namespace crl {

/**
 * A pair of state,reward is referred to as an observation
 */
class _Observation {
protected:
	State _s;
	Reward _r;
public:
	_Observation(const State& s, Reward r)
	: _s(s), _r(r) { }
	virtual ~_Observation() { }

	State getState() {return _s;}
	Reward getReward() {return _r;}
};
typedef boost::shared_ptr<_Observation> Observation;
inline std::ostream& operator<<(std::ostream& os, const Observation& o) {
	os << "o{" << o->getState() << "," << o->getReward() << "}";
	return os;
}

/**
 * \brief An abstract interface for an MDP (implemented elsewhere).
 * Supported are state and action iteration, and reward and transition function retrieval.
 * As per rl-glue nomenclature, an \a Observation is a sampled successor state along with a reward signal.
 */
class _MDP {
public:
	virtual ~_MDP() { }

	virtual StateIterator S() = 0;
	virtual StateIterator predecessors(const State& s) = 0;
	virtual ActionIterator A() = 0;
	virtual ActionIterator A(const State& s) = 0;
	virtual StateDistribution T(const State& s, const Action& a) = 0;
	virtual Reward R(const State& s, const Action& a) = 0;
	virtual Probability T(const State& s, const Action& a, const State& s_next) {
		return T(s, a)->P(s_next);
	}
	///
	/// \brief sample a successor state and reward (i.e., generate an \a Observation)
	///
	virtual Observation sample(const State& s, const Action& a) {
		Observation o = boost::make_shared<_Observation>(T(s, a)->sample(), R(s, a));
		return o;
	}
	virtual void printXML(std::ostream& os);
};
typedef boost::shared_ptr<_MDP> MDP;

/**
 */
class _Heuristic {
public:
	virtual ~_Heuristic() { }
	virtual Reward getPotential(const State& s) = 0;
	virtual Reward getPotential(const State& s, const Action& a) = 0;
};
typedef boost::shared_ptr<_Heuristic> Heuristic;

/**
 * All states have the same heuristic. eg. Rmax.
 */
class _FlatHeuristic : public _Heuristic {
protected:
	Reward _v;
public:
	_FlatHeuristic(Reward v)
	: _v(v) { }
	virtual ~_FlatHeuristic() { }
	virtual Reward getPotential(const State& s) {
		return _v;
	}
	virtual Reward getPotential(const State& s, const Action& a) {
		return _v;
	}
};
typedef boost::shared_ptr<_FlatHeuristic> FlatHeuristic;

/**
 * \brief Interface of the Q-function
 * Supports a heuristic with (s, a) reward estimates
 */
class _QTable {
protected:
	/// \brief Heuristic with (s, a) reward estimates
	Heuristic _potential;
	_QTable() { }
	/// \brief ctor which supports a \a Heuristic for estimating q-values of each state
	_QTable(Heuristic potential)
	: _potential(potential) { }
public:
	virtual ~_QTable() { }

	virtual Reward getQ(const State& s, const Action& a) = 0;
	virtual void setQ(const State& s, const Action& a, Reward r) = 0;
	virtual Action getBestAction(const State& s) = 0;
	virtual Reward getV(const State& s) {
		return getQ(s, getBestAction(s));
	}
	virtual void print(std::ostream& os, StateIterator sitr, ActionIterator aitr);
};
typedef boost::shared_ptr<_QTable> QTable;

/**
 * Interface for a generic planner
 */
class _Planner {
public:
	virtual ~_Planner() { }
	virtual Action getAction(const State& s) = 0;
};
typedef boost::shared_ptr<_Planner> Planner;

/**
 * A simple planner for testing data throughput
 */
class _RandomPlanner : public _Planner {
protected:
	Domain _domain;
public:
	_RandomPlanner(const Domain& domain)
	: _domain(domain) { }
	virtual ~_RandomPlanner() { }
	virtual Action getAction(const State& s) {
		Size numActions = _domain->getNumActions();
		Index actionIndex = random()%numActions;
		Action a(_domain, actionIndex);
		return a;
	}
};
typedef boost::shared_ptr<_RandomPlanner> RandomPlanner;

/**
 * Interface for something that learns from experience.
 */
class _Learner {
public:
	virtual ~_Learner() { }
	/**
	 * Returns true if this observation changed what the learner knows. Allows
	 * agents to only trigger the planner when necessary.
	 */
	virtual bool observe(const State& s, const Action& a, const Observation& o) = 0;
};
typedef boost::shared_ptr<_Learner> Learner;

/**
 * \brief Interface for classes that make observations and take actions
 * An Agent can encapsulate both a \a Planner and \b Learner component.
 */
class _Agent {
protected:
	Planner _planner;
	Learner _learner;
	State _last_state;
	Action _last_action;
public:
	_Agent(Planner planner);
	_Agent(Planner planner, Learner learner);
	virtual ~_Agent() { }

	virtual void begin(const State& s);
	virtual void end();
	///
	/// \brief Observe a transition and reward signal after a previous \a getAction call.
	/// E.g., to perform an iteration of learning.
	///
	virtual bool observe(const Observation& o);
	///
	/// \brief Invoke \a Planner to obtain next action in \a s.
	///
	virtual Action getAction(const State& s);
};
typedef boost::shared_ptr<_Agent> Agent;

/**
 * Interface for an RL environment that starts in one state and generates successor observations
 * given an agent action.
 */
class _Environment {
public:
	virtual ~_Environment() { }

	virtual State begin() = 0;
	virtual bool isTerminated() = 0;
	virtual Observation getObservation(const Action& a) = 0;
};
typedef boost::shared_ptr<_Environment> Environment;

/**
 * The compbination of an agent and an environment along with execution code for the experiment.
 */
class _Experiment {
protected:
	Environment _environment;
	Agent _agent;
	int _num_trials;
public:
	_Experiment(const Environment& environment, const Agent& agent, int num_trials=1);
	Reward runExperiment();
};
typedef boost::shared_ptr<_Experiment> Experiment;

/**
 * Just a combination of the two interfaces.
 * Created for the RMaxMDPLearner, so it could have an underlying MDP and
 * learner be one thing without having to do runtime type checking to verify.
 */
class _MDPLearner : public _MDP, public _Learner {
public:
	virtual ~_MDPLearner() { }
};
typedef boost::shared_ptr<_MDPLearner> MDPLearner;


}

#endif /*CRL_H_*/
