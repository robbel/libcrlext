/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth
    Copyright 2015 Philipp Robbel

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

#ifndef MAZES_HPP_
#define MAZES_HPP_

#include <iostream>
#include <vector>
#include <boost/multi_array.hpp>
#include <boost/shared_ptr.hpp>
#include <crl/crl.hpp>
#include <crl/flat_tables.hpp>
#include <crl/hash_tables.hpp>
#include <strxml.hpp>

namespace crl {

///
/// \brief Defines the layout of a maze
///
class _Maze {
protected:
	size_t _width;
	size_t _height;
	boost::multi_array<char,2> _tiles;
	boost::multi_array<bool,2> _vertical_walls;
	boost::multi_array<bool,2> _horizontal_walls;

public:
	_Maze(size_t width, size_t height);
	virtual ~_Maze() { }
	size_t getWidth() const {return _width;}
	size_t getHeight() const {return _height;}

	bool getWallNorth(size_t x, size_t y);
	bool getWallEast(size_t x, size_t y);
	bool getWallSouth(size_t x, size_t y);
	bool getWallWest(size_t x, size_t y);

	void setWallNorth(size_t x, size_t y, bool wall);
	void setWallEast(size_t x, size_t y, bool wall);
	void setWallSouth(size_t x, size_t y, bool wall);
	void setWallWest(size_t x, size_t y, bool wall);
	char getTile(size_t x, size_t y);
	void setTile(size_t x, size_t y, char tile);
};
typedef boost::shared_ptr<_Maze> Maze;

class _SlipConfig {
protected:
	Probability _slip_forward;
	Probability _slip_left;
	Probability _slip_right;
	Reward _reward_goal;
	Reward _reward_pit;
	Reward _reward_step;
public:
	_SlipConfig();
	_SlipConfig(Probability slip_forward,
			    Probability slip_left,
			    Probability slip_right,
			    Reward reward_goal,
			    Reward reward_pit,
			    Reward reward_step);
	Probability getSlipForward() const {return _slip_forward;}
	Probability getSlipLeft() const {return _slip_left;}
	Probability getSlipRight() const {return _slip_right;}
	Reward getRewardGoal() const {return _reward_goal;}
	Reward getRewardPit() const {return _reward_pit;}
	Reward getRewardStep() const {return _reward_step;}
	void setSlipForward(Probability slip_forward) {_slip_forward=slip_forward;}
	void setSlipLeft(Probability slip_left) {_slip_left=slip_left;}
	void setSlipRight(Probability slip_right) {_slip_right=slip_right;}
	void setRewardGoal(Reward reward_goal) {_reward_goal=reward_goal;}
	void setRewardPit(Reward reward_pit) {_reward_pit=reward_pit;}
	void setRewardStep(Reward reward_step) {_reward_step=reward_step;}
};
typedef boost::shared_ptr<_SlipConfig> SlipConfig;

///
/// \brief A maze where an agent can move fwd,left,right and may slip.
///
class _SlipMaze : public _Maze {
protected:
	Domain _domain;
	SlipConfig _config;
public:
	_SlipMaze(const Maze& maze, const SlipConfig& config);
	virtual ~_SlipMaze() { }
	virtual Domain getDomain();
	virtual MDP getMDP();
	State getTileState(char c);
	State getState(size_t x, size_t y);
	virtual StateSet getTerminalStates();
};
typedef boost::shared_ptr<_SlipMaze> SlipMaze;

class Location {
public:
	int x, y;
	Location(int x, int y)
	: x(x), y(y) { }
};

class _FlagMaze : public _Maze {
public:
protected:
	SlipConfig _config;
	Domain _domain;
	std::vector<Location> _spawns;
	std::vector<Location> _flags;
public:
	_FlagMaze(const Maze& maze, const SlipConfig& config);
	virtual ~_FlagMaze();
	virtual Domain getDomain();
	virtual State getInitialState();
	SlipConfig getConfig() const {return _config;}
	const std::vector<Location>& getFlags() const {return _flags;}
	const std::vector<Location>& getSpawns() const {return _spawns;}
	virtual StateSet getTerminalStates();
};
typedef boost::shared_ptr<_FlagMaze> FlagMaze;

class _FlagMDP : public _MDP {
protected:
	FlagMaze _fm;
	Domain _domain;

	Probability updateDistribution(HStateDistribution& sd,
				       std::vector<Location> locs,
				       std::vector<int> directions,
				       std::vector<int> turns,
				       std::vector<int> flags);
public:
	_FlagMDP(FlagMaze& fm);
	virtual ~_FlagMDP();
	virtual StateIterator S();
	virtual StateIterator predecessors(const State& s);
	virtual ActionIterator A();
	virtual ActionIterator A(const State& s);
	virtual StateDistribution T(const State& s, const Action& a);
	virtual Reward R(const State& s, const Action& a);
	virtual Observation sample(const State& s, const Action& a);
};

Maze readMaze(std::istream& is);
SlipConfig readSlipConfig(std::istream& is);

}

#endif /*MAZES_HPP_*/
