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
	virtual Observation sample(const State& s, const Action& a) {
		Observation o(new _Observation(T(s, a)->sample(), R(s, a)));
		return o;
	}
	virtual void printXML(std::ostream& os);
};
typedef boost::shared_ptr<_MDP> MDP;

class _QTable {
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

class _Learner {
public:
	virtual ~_Learner() { }
	/**
	 * Returns true if this observation changed what the learner knows.
	 */
	virtual bool observe(const State& s, const Action& a, const Observation& o) = 0;
};
typedef boost::shared_ptr<_Learner> Learner; 

class _Heuristic {
public:
	virtual ~_Heuristic() { }
	virtual Reward getPotential(const State& s) = 0;
	virtual Reward getPotential(const State& s, const Action& a) = 0;
};
typedef boost::shared_ptr<_Heuristic> Heuristic;

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
 * Interface for classes that make observations and take actions
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
	virtual bool observe(const Observation& o);
	virtual Action getAction(const State& s);
};
typedef boost::shared_ptr<_Agent> Agent;

class _Environment {
public:
	virtual ~_Environment() { }
	
	virtual State begin() = 0;
	virtual bool isTerminated() = 0;
	virtual Observation getObservation(const Action& a) = 0;
};
typedef boost::shared_ptr<_Environment> Environment;

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

class _MDPLearner : public _MDP, public _Learner {
public:
	virtual ~_MDPLearner() { }
};
typedef boost::shared_ptr<_MDPLearner> MDPLearner;


}

#endif /*CRL_H_*/
