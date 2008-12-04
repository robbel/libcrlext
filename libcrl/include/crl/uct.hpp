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

#include <math.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "crl/crl.hpp"
#include "crl/vi.hpp"
#include "crl/prioritized_sweeping.hpp"
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

#endif /*UCT_HPP_*/
