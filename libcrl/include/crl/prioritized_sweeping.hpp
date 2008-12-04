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

#ifndef PRIORITIZED_SWEEPING_HPP_
#define PRIORITIZED_SWEEPING_HPP_

#include <math.h>
#include <map>
#include <vector>
#include "crl/vi.hpp"

namespace crl {

class _SPriorityQueue {
protected:
	SCountTable _heap_indices;
	struct pnode {
		State s;
		Reward priority;
		pnode(State s, Reward priority)
		: s(s), priority(priority) { }
		pnode() { }
	};
	std::vector<pnode> _heap;
	Size gpi(Size i) {
		return (i-1)/2;	
	}
	Size glci(Size i) {
		return 2*i+1;
	}
	Size grci(Size i) {
		return 2*i+2;
	}
	void swap(Size i1, Size i2);
	void heapUp(Size i);
	void heapDown(Size i);
public:
	_SPriorityQueue(SCountTable heap_indices)
	: _heap_indices(heap_indices) { }
	void insert(State s, Reward priority);
	bool empty();
	State pop();
	Reward peek();
};
typedef boost::shared_ptr<_SPriorityQueue> SPriorityQueue;

class _PSPlanner : public _VIPlanner {
public:
protected:
	SPriorityQueue _pqueue;
	_PSPlanner(const MDP& mdp, Reward epsilon, float gamma);
public:
	_PSPlanner(const MDP& mdp, Reward epsilon, float gamma, QTable qtable, SPriorityQueue pqueue);
	int sweep(ActionIterator& aitr);
	void insert(State s, Reward priority);
};
typedef boost::shared_ptr<_PSPlanner> PSPlanner;

class _FactoredPSPlanner : public _PSPlanner {
public:
	_FactoredPSPlanner(const Domain& domain, const MDP& mdp, Reward epsilon, float gamma)
	: _PSPlanner(mdp, epsilon, gamma) {
		_qtable = FQTable(new _FQTable(domain, 0));
		SCountTable heap_indices = SCountTable(new _FStateTable<Size>(domain, -1));
		_pqueue = SPriorityQueue(new _SPriorityQueue(heap_indices));
	}
};
typedef boost::shared_ptr<_FactoredPSPlanner> FactoredPSPlanner;
	
}

#endif /*PRIORITIZED_SWEEPING_HPP_*/
