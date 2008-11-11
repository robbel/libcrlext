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
 
#include "crl/sparse_sampling.hpp"

using namespace crl;
using namespace std;

_SampleTree::_SampleTree(const MDP& mdp, const State& s, Reward r, int breadth)
: _mdp(mdp), _s(s), _r(r), _breadth(breadth) {
	_best_q = r;
	_best_action = _mdp->A()->next();
}

void _SampleTree::sampleToDepth(int depth, float gamma) {
	if (depth <= 0)
		return;
	
	ActionIterator itr = _mdp->A();
	while (itr->hasNext()) {
		Action a = itr->next();
		
		Reward q = 0;
		for (int i=0; i<_breadth; i++) {
			Observation o = _mdp->sample(_s, a);
			State n = o->getState();
			//cout << "sampled " << _s << ", " << a << " and got " << n << endl;
			Reward r = o->getReward();
			SampleTree subtree = addSample(a, n, r, _breadth);
			subtree->sampleToDepth(depth-1, gamma);
			q = subtree->getV();
		}
		q /= _breadth;
		q *= gamma;
		q += _r;
		
		setQ(a, q);
		
		if (q > _best_q) {
			_best_q = q;
			_best_action = a;
		}
	}
}

std::set<SampleTree>& _MapSampleTree::getSamples(const Action& a) {
	return _tree_map[a];
}

SampleTree _MapSampleTree::addSample(const Action& a, const State& n, Reward r, int breadth) {
	SampleTree subtree(new _MapSampleTree(_mdp, n, r, breadth));
	_tree_map[a].insert(subtree);
	return subtree;
}

void _MapSampleTree::setQ(const Action& a, Reward q) {
	_q_map[a] = q;
}

Reward _MapSampleTree::getQ(const Action& a) {
	return _q_map[a];
}

_MapSampleTree::_MapSampleTree(const MDP& mdp, const State& s, Reward r, int breadth)
: _SampleTree(mdp, s, r, breadth) {
	
}

Action _SparseSamplingPlanner::getAction(const State& s) {
	SampleTree root(new _MapSampleTree(_mdp, s, 0, _breadth));
	root->sampleToDepth(_depth, _gamma);
	cout << "V(" << s << ") = " << root->getV() << endl;
	return root->getBestAction();
}

_SparseSamplingPlanner::_SparseSamplingPlanner(const MDP& mdp, float gamma, int breadth, int depth)
: _mdp(mdp), _gamma(gamma), _breadth(breadth), _depth(depth) {
	
}
