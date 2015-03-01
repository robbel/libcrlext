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

#include <cmath>
#include <map>
#include <vector>
#include "crl/vi.hpp"

namespace crl {

/**
 * Used internally for _PSPlanner. An updateable priority queue
 * for states
 */
class _SPriorityQueue {
protected:
	/**
	 * a table to keep track of where in the heap a state's
	 * priority is kept
	 */
	SCountTable _heap_indices;
	/**
	 * An element in the heap array
	 */
	struct pnode {
		State s;
		Reward priority;
		pnode(State s, Reward priority)
		: s(s), priority(priority) { }
		pnode() { }
	};
	/**
	 * Array representation of a full binary tree
	 */
	std::vector<pnode> _heap;
	/**
	 * index of i's parent
	 */
	Size gpi(Size i) {
		return (i-1)/2;
	}
	/**
	 * index of i's left child
	 */
	Size glci(Size i) {
		return 2*i+1;
	}
	/**
	 * index of i's right child
	 */
	Size grci(Size i) {
		return 2*i+2;
	}
	/**
	 * Swap two elements in the heap, updating their indices
	 */
	void swap(Size i1, Size i2);
	/**
	 * push an element up the heap as far as it can go
	 */
	void heapUp(Size i);
	/**
	 * push an element down the heap as far as it can go
	 */
	void heapDown(Size i);
public:
	_SPriorityQueue(SCountTable heap_indices)
	: _heap_indices(heap_indices) { }
	/**
	 * set the priority for a state
	 */
	void insert(State s, Reward priority);
	bool empty();
	/**
	 * Extract the state with highest priority
	 */
	State pop();
	/**
	 * peek at the highest priority
	 */
	Reward peek();
};
typedef boost::shared_ptr<_SPriorityQueue> SPriorityQueue;

/**
 * A planner that does prioritized sweeping
 */
class _PSPlanner : public _VIPlanner {
public:
protected:
	SPriorityQueue _pqueue;
	_PSPlanner(MDP& mdp, Reward epsilon, float gamma);
public:
	_PSPlanner(MDP& mdp, Reward epsilon, float gamma, QTable qtable, SPriorityQueue pqueue);
	/**
	 * perform a sweep with some set of actions
	 */
	int sweep(ActionIterator& aitr);
	/**
	 * perform a sweep with the actions from _mdp->A()
	 */
	int sweep();
	/**
	 * set the priority of some state
	 */
	void insert(State s, Reward priority);
	/**
	 * set the priority of some state to the max 1-step reward
	 */
	void insert(State s);
	/**
	 * set the priority of a bunch of states to their max 1-step reward
	 */
	void insert(StateIterator& sitr);
	/**
	 * set the priority of a bunch of states to their max 1-step reward
	 * only if they exceed a threshold
	 */
	void insertThreshold(StateIterator& sitr, Reward theshold);
	/**
	 * get the current best state
	 */
	virtual Action getAction(const State& s);
};
typedef boost::shared_ptr<_PSPlanner> PSPlanner;

/**
 * Instantiate a PS planner using flat tables
 */
class _FlatPSPlanner : public _PSPlanner {
public:
	_FlatPSPlanner(const Domain& domain, MDP& mdp, Reward epsilon, float gamma)
	: _PSPlanner(mdp, epsilon, gamma) {
		_qtable = FQTable(new _FQTable(domain, 0));
		SCountTable heap_indices = SCountTable(new _FStateTable<Index>(domain, -1));
		_pqueue = SPriorityQueue(new _SPriorityQueue(heap_indices));
	}
};
typedef boost::shared_ptr<_FlatPSPlanner> FlatPSPlanner;

}

#endif /*PRIORITIZED_SWEEPING_HPP_*/
