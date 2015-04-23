/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth

    This file is part of mazes.

    mazes is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    mazes is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with strxml.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <sstream>
#include <strxml.hpp>
#include <cpputil.hpp>
#include <crl/flat_tables.hpp>
#include "crl/mazes.hpp"

using namespace std;
using namespace boost;
using namespace cpputil;
using namespace crl;

_FlagMaze::_FlagMaze(const Maze& maze, const SlipConfig& config)
: _Maze(*maze), _config(config), _domain(new _Domain()) {
//	_domain = Domain(new _Domain());

	Reward min_r = _config->getRewardPit();
	if (_config->getRewardStep() < min_r)
		min_r = _config->getRewardStep();
	if (_config->getRewardGoal() < min_r)
		min_r = _config->getRewardGoal();
	Reward max_r = _config->getRewardPit();
	if (_config->getRewardStep() > max_r)
		max_r = _config->getRewardStep();
	if (_config->getRewardGoal() > max_r)
		max_r = _config->getRewardGoal();
	_domain->setRewardRange(min_r, max_r);

	_domain->addStateFactor(0, 1);//purgatory
	for (size_t x=0; x<getWidth(); x++)
		for (size_t y=0; y<getHeight(); y++) {
			if (getTile(x, y) == 'S') {
				//one collecter per 'S'
				_spawns.push_back(Location(x, y));
				_domain->addStateFactor(0, getWidth()-1);//gets an X
				_domain->addStateFactor(0, getHeight()-1);// and a Y
				_domain->addActionFactor(0, 3);//and an action
			}
		}
	for (size_t x=0; x<getWidth(); x++)
		for (size_t y=0; y<getHeight(); y++) {
			if (getTile(x, y) == 'F') {
				_flags.push_back(Location(x, y));
				//each flag gets an indicator
				_domain->addStateFactor(0, 1);
			}
		}
}
_FlagMaze::~_FlagMaze() {

}
crl::Domain _FlagMaze::getDomain() {
	return _domain;
}

State _FlagMaze::getInitialState() {
	State s(_domain, 0);
	s.setFactor(0, 0);
	for (size_t i=0; i<_spawns.size(); i++) {
		s.setFactor(1+2*i, _spawns[i].x);
		s.setFactor(2+2*i, _spawns[i].y);
	}
	for (size_t i=0; i<_flags.size(); i++) {
		s.setFactor(1+2*(_spawns.size())+i, 0);
	}
	return s;
}

set<vector<int> > getFlagOrderings(int collectors_left, set<int> flags_left) {
	set<vector<int> > sv;
	if (collectors_left == 0) {
		sv.insert(vector<int>());
		return sv;
	}

	for (set<int>::iterator itr=flags_left.begin(); itr!=flags_left.end(); ++itr) {
		int flag = *itr;
		set<int> next_flags_left(flags_left);
		next_flags_left.erase(next_flags_left.find(flag));
		set<vector<int> > next_orderings(getFlagOrderings(collectors_left-1, next_flags_left));
		for (set<vector<int> >::iterator itr=next_orderings.begin(); itr!=next_orderings.end(); ++itr) {
			vector<int> ordering(*itr);
			ordering.insert(ordering.begin(), flag);
			sv.insert(ordering);
		}
	}

	return sv;
}

/**
 * Not all terminal states, only some of the useful ones where a collector stops
 * after collecting its last flag. Obvious sometimes multiple collectors would go
 * for the last flag, in case something goes wrong for the closest one, and that
 * ending wouldn't appear here.
 */
StateSet _FlagMaze::getTerminalStates() {
	StateSet ss(new _StateSet());
	set<int> flags_left;
	for (size_t i=0; i<_flags.size(); i++)
		flags_left.insert(i);
	set<vector<int> > flag_orderings = getFlagOrderings(_spawns.size(), flags_left);
	for (set<vector<int> >::iterator itr=flag_orderings.begin(); itr!=flag_orderings.end(); ++itr) {
		vector<int> flag_order = *itr;
		State s(_domain, 0);
		s.setFactor(0, 0);
		for (size_t i=0; i<flag_order.size(); i++) {
			int f = flag_order[i];
			s.setFactor(1+2*i, _flags[f].x);
			s.setFactor(2+2*i, _flags[f].y);
		}
		for (Size i=0; i<_flags.size(); i++)
			s.setFactor(1+2*_spawns.size()+i, 1);
		ss->insert(s);
	}

	return ss;
}


_FlagMDP::_FlagMDP(FlagMaze& fm)
: _fm(fm), _domain(_fm->getDomain()) {

}

_FlagMDP::~_FlagMDP() {

}
StateIterator _FlagMDP::S() {
	Domain domain = _fm->getDomain();
	StateIterator itr(new _StateIncrementIterator(domain));
	return itr;
}
StateIterator _FlagMDP::predecessors(const State& s) {
	StateSet ss(new _StateSet());


	if (s.getFactor(0) == 1) {
		State p = s;
		p.setFactor(0, 0);
		ss->insert(p);
		SharedStateSetIterator itr(new _SharedStateSetIterator(ss));
		return itr;
	}

	vector<Location> flagLocs = _fm->getFlags();
	vector<Location> spawns = _fm->getSpawns();
	vector<Location> locs;
	vector<int> directions;
	vector<int> flags;

	for (size_t i=0; i<flagLocs.size(); i++) {
		flags.push_back(s.getFactor(1+2*spawns.size()+i));
	}

	for (size_t i=0; i<spawns.size(); i++) {
		int x = s.getFactor(1+2*i);
		int y = s.getFactor(2+2*i);
		locs.push_back(Location(x, y));
		directions.push_back(0);
	}

	while (true) {
		bool all_3 = true;
		for (size_t i=0; all_3 && i<directions.size(); i++)
			all_3 = all_3 && (directions[i]==3);
		if (all_3)
			break;

		vector<int> pre_flags;
		for (size_t i=0; i<flagLocs.size(); i++)
			pre_flags.push_back(0);
		while (true) {

			bool all_1 = true;
			for (size_t i=0; all_1 && i<pre_flags.size(); i++)
				all_1 = all_1 && (pre_flags[i]==1);
			if (all_1)
				break;

			bool good_flags = true;
			for (size_t i=0; good_flags && i<pre_flags.size(); i++)
				if (pre_flags[i] == 1 && flags[i] == 0)
					good_flags = false;

			if (good_flags) {
				State p = s;
				for (size_t i=0; i<locs.size(); i++) {
					int x = locs[i].x;
					int y = locs[i].y;
					if (directions[i] == 0 && !_fm->getWallNorth(x, y))
						y--;
					if (directions[i] == 1 && !_fm->getWallEast(x, y))
						x++;
					if (directions[i] == 2 && !_fm->getWallSouth(x, y))
						y++;
					if (directions[i] == 3 && !_fm->getWallWest(x, y))
						x--;
					p.setFactor(1+2*i, x);
					p.setFactor(2+2*i, y);
				}
				for (size_t i=0; i<flags.size(); i++) {
					p.setFactor(1+2*locs.size()+i, pre_flags[i]);
				}
				ss->insert(p);
			}

			for (size_t i=0; i<pre_flags.size(); i++) {
				pre_flags[i] += 1;
				if (pre_flags[i] > 1)
					pre_flags[i] = 0;
				else
					break;
			}
		}

		for (size_t i=0; i<directions.size(); i++) {
			directions[i] += 1;
			if (directions[i] > 3)
				directions[i] = 0;
			else
				break;
		}
	}

	SharedStateSetIterator itr(new _SharedStateSetIterator(ss));
	return itr;
}
ActionIterator _FlagMDP::A() {
	ActionIterator itr(new _ActionIncrementIterator(_fm->getDomain()));
	return itr;
}
ActionIterator _FlagMDP::A(const State& s) {
	if (s.getFactor(0) == 0)
		return A();
	EmptyActionIterator itr(new _EmptyActionIterator());
	return itr;
}


Probability _FlagMDP::updateDistribution(HStateDistribution& sd,
								 	     vector<Location> locs,
                                 		 vector<int> directions,
                                 		 vector<int> turns,
								 		 vector<int> flags) {

	vector<Location> flagLocs = _fm->getFlags();
	vector<Location> spawns = _fm->getSpawns();
	SlipConfig config = _fm->getConfig();
	Probability p = 1;
	for (size_t i=0; i<turns.size(); i++) {
		if (turns[i] == -1)
			p *= config->getSlipLeft();
		if (turns[i] == 0)
			p *= config->getSlipForward();
		if (turns[i] == 1)
			p *= config->getSlipRight();
	}
	for (size_t i=0; i<turns.size(); i++) {
		directions[i] += turns[i];
		if (directions[i] < 0)
			directions[i] += 4;
		if (directions[i] > 3)
			directions[i] -= 4;
	}

	vector<Location> dsts;
	for (size_t i=0; i<locs.size(); i++) {
		int x = locs[i].x;
		int y = locs[i].y;
		//collectors in pits cannot move
		if (_fm->getTile(x, y) == '#')
			continue;
		if (directions[i] == 0 && !_fm->getWallNorth(x, y))
			y--;
		if (directions[i] == 1 && !_fm->getWallEast(x, y))
			x++;
		if (directions[i] == 2 && !_fm->getWallSouth(x, y))
			y++;
		if (directions[i] == 3 && !_fm->getWallWest(x, y))
			x--;
		dsts.push_back(Location(x, y));
		for (size_t j=0; j<flags.size(); j++) {
			if (!flags[j] && x == flagLocs[j].x && y == flagLocs[j].y)
				flags[j] = 1;
		}
	}

	State s(_fm->getDomain());
	s.setFactor(0, 0);

	for (size_t i=0; i<dsts.size(); i++) {
		s.setFactor(1+2*i, dsts[i].x);
		s.setFactor(2+2*i, dsts[i].y);
	}
	for (size_t i=0; i<flags.size(); i++) {
		s.setFactor(1+2*(spawns.size())+i, flags[i]);
	}
	sd->setP(s, p+sd->P(s));

	return p;
}

StateDistribution _FlagMDP::T(const State& s, const Action& a) {
	vector<Location> flagLocs = _fm->getFlags();
	vector<Location> spawns = _fm->getSpawns();
	HStateDistribution sd(new _HStateDistribution(_domain));
	if (s.getFactor(0) == 1) {
		sd->setP(s, 1);
		return sd;
	}

	vector<Location> locs;
	vector<int> directions;
	vector<int> turn_stack;
	vector<int> flags;

	for (size_t i=0; i<spawns.size(); i++) {
		int x = s.getFactor(1+2*i);
		int y = s.getFactor(2+2*i);
		locs.push_back(Location(x, y));
		directions.push_back(a.getFactor(i));
		turn_stack.push_back(-1);
	}

	bool all_flags = true;

	for (size_t i=0; i<flagLocs.size(); i++) {
		int f = s.getFactor(1+2*(spawns.size())+i);
		all_flags = all_flags && f;
		flags.push_back(f);
	}

	if (all_flags) {
		State n = s;
		n.setFactor(0, 1);
		sd->setP(n, 1);
		return sd;
	}
	Probability c = 0;
	while (true) {

		c += updateDistribution(sd, locs, directions, turn_stack, flags);
		bool all_1 = true;
		for (size_t i=0; all_1 && i<turn_stack.size(); i++)
			all_1 = all_1 && (turn_stack[i]==1);
		if (all_1)
			break;

		for (size_t i=0; i<turn_stack.size(); i++) {
			turn_stack[i] += 1;
			if (turn_stack[i] > 1)
				turn_stack[i] = -1;
			else
				break;
		}
	}

	c = 0;
	StateIterator itr = sd->iterator();
	while (itr->hasNext()) {
		c += sd->P(itr->next());
	}

	return sd;
}
Reward _FlagMDP::R(const State& s, const Action& a) {
	if (s.getFactor(0) == 1)
		return 0;
	vector<Location> flagLocs = _fm->getFlags();
	vector<Location> spawns = _fm->getSpawns();
	for (size_t i=0; i<flagLocs.size(); i++) {
		size_t j = i+1+2*spawns.size();
		if (s.getFactor(j) == 0)
			return _fm->getConfig()->getRewardStep();
	}
	return _fm->getConfig()->getRewardGoal();
}
Observation _FlagMDP::sample(const State& s, const Action& a) {
	if (s.getFactor(0) == 1) {
		Observation o(new _Observation(s, 0));
		return o;
	}

	vector<Location> flagLocs = _fm->getFlags();
	vector<Location> spawns = _fm->getSpawns();
	SlipConfig config = _fm->getConfig();

	bool all_flags = true;

	for (size_t i=0; i<flagLocs.size(); i++) {
		int f = s.getFactor(1+2*(spawns.size())+i);
		all_flags = all_flags && f;
	}

	if (all_flags) {
		State n = s;
		n.setFactor(0, 1);
		Observation o(new _Observation(n, config->getRewardGoal()));
		return o;
	}

	State n = s;

	for (size_t i=0; i<spawns.size(); i++) {
		int direction = a.getFactor(i);
		int x = s.getFactor(1+2*i);
		int y = s.getFactor(2+2*i);
		Probability p = randDouble();
		int turn = 0;
		if (p < config->getSlipLeft())
			turn = -1;
		else if (p < config->getSlipLeft()+config->getSlipRight())
			turn = 1;

		direction += turn;
		if (direction < 0)
			direction += 4;
		if (direction > 3)
			direction -= 4;
		if (direction == 0 && !_fm->getWallNorth(x, y))
			y--;
		if (direction == 1 && !_fm->getWallEast(x, y))
			x++;
		if (direction == 2 && !_fm->getWallSouth(x, y))
			y++;
		if (direction == 3 && !_fm->getWallWest(x, y))
			x--;
		n.setFactor(1+2*i, x);
		n.setFactor(2+2*i, y);

		for (size_t j=0; j<flagLocs.size(); j++) {
			if (x == flagLocs[j].x && y == flagLocs[j].y) {
				n.setFactor(1+2*(spawns.size())+i, 1);
			}
		}
	}

	Observation o(new _Observation(n, config->getRewardStep()));
	return o;
}
