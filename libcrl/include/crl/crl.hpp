/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth
    Copyright 2015 Philipp Robbel

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

#ifndef CRL_HPP_
#define CRL_HPP_

#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <boost/shared_ptr.hpp>
#include <cpputil.hpp>
#include "crl/util.hpp"
#include "crl/common.hpp"
#include "crl/bigindex.hpp"

namespace crl {

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
	virtual Reward getPotential(const State& s) override {
		return _v;
	}
	virtual Reward getPotential(const State& s, const Action& a) override {
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
 * A policy has the same interface as the \a Planner above
 */
typedef _Planner _Policy;
typedef boost::shared_ptr<_Policy> Policy;

/**
 * A simple policy (no planning) for testing data throughput
 */
class _RandomPolicy : public _Policy {
protected:
	Domain _domain;
public:
	_RandomPolicy(const Domain& domain)
	: _domain(domain) { }
	virtual ~_RandomPolicy() { }
	virtual Action getAction(const State& s) override {
		Size numActions = _domain->getNumActions();
		Index actionIndex = random()%numActions;
		Action a(_domain, actionIndex);
		return a;
	}
};
typedef boost::shared_ptr<_RandomPolicy> RandomPolicy;

/**
 * A simple policy (no planning) that always chooses 0 (no action)
 */
class _NullPolicy : public _Policy {
protected:
	Domain _domain;
public:
	_NullPolicy(const Domain& domain)
	: _domain(domain) { }
	virtual ~_NullPolicy() { }
	virtual Action getAction(const State& s) override {
		return Action(_domain,0);
	}
};
typedef boost::shared_ptr<_NullPolicy> NullPolicy;

/**
 * Interface for something that learns from experience.
 * \note State and Observation are polymorphic types (e.g., also BigState, BigObservation)
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
 * \note State and Observation are polymorphic types (e.g., also BigState, BigObservation)
 */
class _Agent {
protected:
	Planner _planner;
	Learner _learner;
	struct {
	  State _s;
	  BigState _big_s;
	} stateImpl;
	State& _last_state;
	Action _last_action;
public:
	_Agent(const Domain& domain, Planner planner);
	_Agent(const Domain& domain, Planner planner, Learner learner);
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
 * An agent that uses a given (pre-computed) policy to act, no learning or planning happens.
 */
class _PolicyAgent : public _Agent {
public:
    _PolicyAgent(const Domain& domain, Policy policy)
    : _Agent(domain, policy) { }
    virtual ~_PolicyAgent() { }
    virtual bool observe(const Observation& o) override {
        return false;
    }
};

/**
 * Interface for an RL environment that starts in one state and generates successor observations
 * given an agent action.
 */
class _Environment {
public:
        typedef State state_t;
        typedef Observation observation_t;

        virtual ~_Environment() { }

	virtual State begin() = 0;
	virtual bool isTerminated() = 0;
	virtual Observation getObservation(const Action& a) = 0;
};
typedef boost::shared_ptr<_Environment> Environment;

/**
 * The compbination of an agent and an environment along with execution code for the experiment.
 * Currently does not use a \a StateMapper as the glue-crl version.
 * \note Supports both _Environment and _BigEnvironment as template parameters.
 */
template<class T>
class _ExperimentBase {
protected:
	boost::shared_ptr<T> _environment;
	Agent _agent;
	int _num_trials;
public:
        _ExperimentBase(const boost::shared_ptr<T>& environment, const Agent& agent, int num_trials=1)
        : _environment(environment), _agent(agent), _num_trials(num_trials) { }

        Reward runExperiment() {
          Reward total = 0;
          for (int trial=0; trial<_num_trials; trial++) {
  //		cout << "******" << endl;
                  typename T::state_t s = _environment->begin();
                  _agent->begin(s);
                  while (!_environment->isTerminated()) {
  //			cout << " step" << endl;
                          Action a = _agent->getAction(s);
                          typename T::observation_t o = _environment->getObservation(a);
  //			cout << s << ", " << a << " -> " << o << endl;
                          s = o->getState();
                          total += o->getReward();
                          _agent->observe(o);
                  }
                  _agent->end();
          }
          return total;
        }
};
typedef _ExperimentBase<_Environment> _Experiment;
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

#endif /*CRL_HPP_*/
