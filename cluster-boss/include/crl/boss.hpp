/*
    Copyright 2009 Rutgers University
    Copyright 2009 John Asmuth

    This file is part of CRL:RL-Glue:bayes.

    CRL:RL-Glue:bayes is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CRL:RL-Glue:bayes is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CRL:RL-Glue:bayes.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef BOSS_HPP_
#define BOSS_HPP_

#include <set>
#include <boost/shared_ptr.hpp>
#include <crl/crl.hpp>
#include <crl/vi.hpp>

namespace crl {

class _BOSSPlanner : public _VIPlanner {
protected:
	std::set<MDP> _mdps;
	_BOSSPlanner(Reward epsilon, float gamma);
public:
	_BOSSPlanner(Reward epsilon, float gamma, QTable qtable);
	virtual ~_BOSSPlanner() { }
	virtual Reward evaluateStateAction(const State& s, const Action& a);
	virtual void setMDPs(std::set<MDP> mdps);
	virtual std::set<MDP> getMDPs() {return _mdps;}
	virtual void plan();
};
typedef boost::shared_ptr<_BOSSPlanner> BOSSPlanner;

};

#endif /* BOSS_HPP_ */
