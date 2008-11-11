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

#include <map>
#include "crl/vi.hpp"

namespace crl {
	
class _PSPlanner : public _VIPlanner {
public:
	typedef std::multimap<Reward,State> priority_queue;
	typedef priority_queue::iterator iterator;
protected:
	
	_PSPlanner(const MDP& mdp, Reward epsilon, float gamma);
public:
	_PSPlanner(const MDP& mdp, Reward epsilon, float gamma, QTable qtable);
	int sweep(priority_queue& pq, ActionIterator& aitr);
};
typedef boost::shared_ptr<_PSPlanner> PSPlanner;

class _FactoredPSPlanner : public _PSPlanner {
public:
	_FactoredPSPlanner(const Domain& domain, const MDP& mdp, Reward epsilon, float gamma)
	: _PSPlanner(mdp, epsilon, gamma) {
		_qtable = FQTable(new _FQTable(domain, 0));
	}
};
typedef boost::shared_ptr<_FactoredPSPlanner> FactoredPSPlanner;
	
}

#endif /*PRIORITIZED_SWEEPING_HPP_*/
