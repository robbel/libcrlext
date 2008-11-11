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
 
#ifndef MDOMAIN_HPP_
#define MDOMAIN_HPP_

#include <map>
#include <sstream>
#include "crl/crl.hpp"

namespace crl {

class _MQTable : public _QTable {
protected:
	typedef std::map<Action,Reward> ARMap;
	typedef std::map<State,ARMap> SARMap;

	Action _null_action;
	Reward _initial;
	SARMap _q_map;
public:
	_MQTable(Reward initial)
	: _initial(initial) { }
	virtual Reward getQ(const State& s, const Action& a) {
		SARMap::iterator sitr
		 = _q_map.find(s);
		if (sitr == _q_map.end()) {
			return _q_map[s][a] = _initial;
		}
		ARMap::iterator aitr = (sitr->second).find(a);
		if (aitr == (sitr->second).end()) {
			return _q_map[s][a] = _initial;
		}
		return _q_map[s][a];
	}
	virtual void setQ(const State& s, const Action& a, Reward r) {
		_q_map[s][a] = r;
	}
	virtual Action getBestAction(const State& s);
};
typedef boost::shared_ptr<_MQTable> MQTable;

template <class T>
class _MDistribution : public _Distribution<T> {
	std::map<T,Probability> prob_map;
public:
	typedef boost::shared_ptr<cpputil::Iterator<T> > Iterator;
	virtual Iterator iterator() {
		Iterator itr(new cpputil::MapKeyIterator<T, std::map<T,Probability> >(prob_map));
		return itr;
	}
	virtual Probability P(const T& t) {
		typename std::map<T,Probability>::iterator itr = prob_map.find(t);
		if (itr == prob_map.end())
			return 0;
		return itr->second;
	}
	virtual T sample() {
		Probability c = 0;
		Probability i = cpputil::randDouble();
		Iterator itr = iterator();
		while (itr->hasNext()) {
			T t = itr->next();
			c += P(t);
			if (c >= i)
				return t;
		}
		std::ostringstream os;
		os << i << "/" << c << std::endl;
		throw DistributionException(os.str());
	}
	void setP(const T& t, Probability p) {
		prob_map[t] = p;
	}
	void clear() {
		prob_map.clear();	
	}
};
typedef _MDistribution<State> _MStateDistribution;
typedef _MDistribution<Action> _MActionDistribution;
typedef boost::shared_ptr<_MStateDistribution> MStateDistribution;
typedef boost::shared_ptr<_MActionDistribution> MActionDistribution;

class _MapMDP : public _MDP {
protected:
	std::map<const State, std::map<const Action, StateDistribution> > _T_map;
	std::map<const State, std::map<const Action, Reward> > _R_map;
	std::set<State> _known_states;
	std::set<Action> _known_actions;
	std::map<const State, _ActionSet> _action_map;
	StateDistribution _empty_T;
public:
	_MapMDP() {
		_empty_T = StateDistribution(new _MStateDistribution());	
	}
	virtual StateIterator S() {
		return StateSetIterator(new _StateSetIterator(_known_states));
	}
	virtual ActionIterator A() {
		return ActionSetIterator(new _ActionSetIterator(_known_actions));
	}
	virtual ActionIterator A(const State& s) {
		return ActionIterator(new _ActionSetIterator(_action_map[s]));
	}
	virtual StateDistribution T(const State& s, const Action& a) {
		if (_T_map.find(s) == _T_map.end() || _T_map[s].find(a) == _T_map[s].end())
			return _empty_T;
		return _T_map[s][a];
	}
	virtual Reward R(const State& s, const Action& a) {
		if (_T_map.find(s) == _T_map.end() || _T_map[s].find(a) == _T_map[s].end())
			return 0;
		return _R_map[s][a];	
	}
	virtual void setT(const State& s, const Action& a, const State& n, Probability p);
	virtual void setR(const State& s, const Action& a, Reward r);
	virtual void clear(const State& s, const Action& a);
	virtual void clear();
};
typedef boost::shared_ptr<_MapMDP> MapMDP;
	
}

#endif /*MDP_HPP_*/
