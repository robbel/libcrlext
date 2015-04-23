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
 
#ifndef SPARSE_SAMPLING_HPP_
#define SPARSE_SAMPLING_HPP_

#include <set>
#include <boost/shared_ptr.hpp>
#include "crl/crl.hpp"

namespace crl {

class _SampleTree;
typedef boost::shared_ptr<_SampleTree> SampleTree;
class _SampleTree {
protected:
	const MDP _mdp;
	
	const State _s;
	Reward _r;
	int _breadth;
	
	Action _best_action;
	Reward _best_q;
	
	virtual std::set<SampleTree>& getSamples(const Action& a) = 0;
	virtual SampleTree addSample(const Action& a, const State& n, Reward r, int breadth) = 0;
	virtual void setQ(const Action& a, Reward q) = 0;
	virtual Reward getQ(const Action& a) = 0;
public:
	_SampleTree(const MDP& mdp, const State& s, Reward r, int breadth);
	virtual ~_SampleTree() { }
	virtual void sampleToDepth(int depth, float gamma);
	Reward getV() const {
		return _best_q;
	}
	Action getBestAction() const {
		return _best_action;
	}
};

class _MapSampleTree : public _SampleTree {
protected:
	std::map<Action,std::set<SampleTree> > _tree_map;
	std::map<Action,Reward> _q_map;

	virtual std::set<SampleTree>& getSamples(const Action& a);
	virtual SampleTree addSample(const Action& a, const State& n, Reward r, int breadth);
	virtual void setQ(const Action& a, Reward q);
	virtual Reward getQ(const Action& a);
	
public:
	_MapSampleTree(const MDP& mdp, const State& s, Reward r, int breadth);
	virtual ~_MapSampleTree() { }
};

class _SparseSamplingPlanner : public _Planner { 	
protected:
	const MDP _mdp;
	float _gamma;
	int _breadth;
	int _depth;
public:
	_SparseSamplingPlanner(const MDP& mdp, float gamma, int breadth, int depth);
	virtual Action getAction(const State& s);
};
typedef boost::shared_ptr<_SparseSamplingPlanner> SparseSamplingPlanner;

}

#endif /*SPARSE_SAMPLING_HPP_*/
