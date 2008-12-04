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
 
#include "crl/prioritized_sweeping.hpp"

using namespace std;
using namespace crl;

void _SPriorityQueue::swap(Size i1, Size i2) {
	pnode tmp = _heap[i1];
	_heap[i1] = _heap[i2];
	_heap[i2] = tmp;
	_heap_indices->setValue(_heap[i1].s, i1);
	_heap_indices->setValue(_heap[i2].s, i2);
}
void _SPriorityQueue::heapUp(Size i) {
	if (i == 0)
		return;
	Size pi = gpi(i);
	if (_heap[pi].priority < _heap[i].priority) {
		swap(i, pi);
		heapUp(pi);
	}
}
void _SPriorityQueue::heapDown(Size i) {
	Size rci = grci(i);
	Size sci = glci(i);
	if (sci >= _heap.size())
		return;
	if (rci < _heap.size() && _heap[sci].priority > _heap[rci].priority)
		sci = rci;
	if (_heap[sci].priority > _heap[i].priority) {
		swap(i, sci);
		heapDown(sci);
	}
}
	
void _SPriorityQueue::insert(State s, Reward priority) {
	priority = fabs(priority);
	Size cur_index = _heap_indices->getValue(s);
	if (cur_index > 0 && s == _heap[cur_index].s) {
		_heap[cur_index].priority += priority;
	}
	else {
		cur_index = _heap.size();
		_heap.push_back(pnode(s, priority));
	}
	heapUp(cur_index);
}
bool _SPriorityQueue::empty() {
	return _heap.size()>0;
}
State _SPriorityQueue::pop() {
	State s = _heap[0].s;
	if (_heap.size() > 1) {
		_heap[0] = _heap[_heap.size()-1];
		heapDown(0);
	}
	_heap.resize(_heap.size()-1);
	_heap_indices->setValue(s, -1);
	return s;
}
Reward _SPriorityQueue::peek() {
	return _heap[0].priority;
}

_PSPlanner::_PSPlanner(const MDP& mdp, Reward epsilon, float gamma)
: _VIPlanner(mdp, epsilon, gamma) {
	
}

_PSPlanner::_PSPlanner(const MDP& mdp, Reward epsilon, float gamma, QTable qtable, SPriorityQueue pqueue)
: _VIPlanner(mdp, epsilon, gamma, qtable), _pqueue(pqueue) {
	
}

int _PSPlanner::sweep(ActionIterator& aitr) {
	int count = 0;
	while (!_pqueue->empty() && _gamma*_pqueue->peek() > _epsilon) {
		count++;
		State s = _pqueue->pop();
		StateIterator sitr = _mdp->predecessors(s);
		while (sitr->hasNext()) {
			State p = sitr->next();
			Reward error = backupState(p, aitr);
			insert(p, error);	
		}
	}
	return count;
}

void _PSPlanner::insert(State s, Reward priority) {
	_pqueue->insert(s, priority);
}
