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
#include <crl/fdomain.hpp>
#include <crl/mdomain.hpp>
#include "crl/mazes.hpp"

using namespace std;
using namespace boost;
using namespace cpputil;
using namespace crl;


_Maze::_Maze(size_t width, size_t height)
: _width(width), _height(height),
  _tiles(extents[width][height]),
  _vertical_walls(extents[width-1][height]),
  _horizontal_walls(extents[width][height-1]) {
	
}

bool _Maze::getWallEast(size_t x, size_t y) {
	if (x<0 || x>=_width-1 || y<0 || y>=_height)
		return true;
	return _vertical_walls[x][y];
}
bool _Maze::getWallSouth(size_t x, size_t y) {
	if (x<0 || x>=_width || y<0 || y>=_height-1)
		return true;
	return _horizontal_walls[x][y];
}
void _Maze::setWallEast(size_t x, size_t y, bool wall) {
	if (x<0 || x>=_width-1 || y<0 || y>=_height)
		return;
	_vertical_walls[x][y] = wall;
}
void _Maze::setWallSouth(size_t x, size_t y, bool wall) {
	if (x<0 || x>=_width || y<0 || y>=_height-1)
		return;
	_horizontal_walls[x][y] = wall;
}
bool _Maze::getWallNorth(size_t x, size_t y) {
	return getWallSouth(x, y-1);
}

bool _Maze::getWallWest(size_t x, size_t y) {
	return getWallEast(x-1, y);
}

void _Maze::setWallNorth(size_t x, size_t y, bool wall) {
	setWallSouth(x, y-1, wall);
}

void _Maze::setWallWest(size_t x, size_t y, bool wall) {
	setWallEast(x-1, y, wall);
}

char _Maze::getTile(size_t x, size_t y) {
	if (x<0 || x>=_width || y<0 || y>=_height)
		return '#';
	return _tiles[x][y];
}

void _Maze::setTile(size_t x, size_t y, char tile) {
	if (x<0 || x>=_width || y<0 || y>=_height)
		return;
	_tiles[x][y] = tile;
}

_SlipConfig::_SlipConfig() 
: _slip_forward(1), _slip_left(0), _slip_right(0),
  _reward_goal(1), _reward_pit(-1), _reward_step(0) {
	
}

_SlipConfig::_SlipConfig(crl::Probability slip_forward,
					     crl::Probability slip_left,
					     crl::Probability slip_right,
					     crl::Reward reward_goal,
					     crl::Reward reward_pit,
					     crl::Reward reward_step) 
: _slip_forward(slip_forward), _slip_left(slip_left), _slip_right(slip_right),
  _reward_goal(reward_goal), _reward_pit(reward_pit), _reward_step(reward_step) {
	
}

		    
_SlipMaze::_SlipMaze(const Maze& maze, const SlipConfig& config)
: _Maze(*maze), _config(config) {
  	
	_domain = Domain(new _Domain());
	_domain->addStateFactor(0, getWidth()-1);
	_domain->addStateFactor(0, getHeight()-1);
	_domain->addStateFactor(0, 1);
	_domain->addActionFactor(0, 3);
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
}

Domain _SlipMaze::getDomain() {
	return _domain;
}

MDP _SlipMaze::getMDP() {
	FMDP mdp(new _FMDP(_domain));
	for (size_t x=0; x<getWidth(); x++)
		for (size_t y=0; y<getHeight(); y++) {
			State s(_domain);
			s.setFactor(0, x);
			s.setFactor(1, y);
			s.setFactor(2, 0);
			
			if (getTile(x, y) == 'G') {
				State purg = s;
				purg.setFactor(2, 1);
				for (int i=0; i<4; i++) {
					Action a(_domain);
					a.setFactor(0, i);
					mdp->setT(s, a, purg, 1);
					mdp->setR(s, a, _config->getRewardGoal());
				}
				continue;	
			}
			
			if (getTile(x, y) == '#') {
				State purg = s;
				purg.setFactor(2, 1);
				for (int i=0; i<4; i++) {
					Action a(_domain);
					a.setFactor(0, i);
					mdp->setT(s, a, purg, 1);
					mdp->setR(s, a, _config->getRewardPit());
				}
				continue;	
			}
			
			State s_n = s;
			s_n.setFactor(1, y-1);
			State s_e = s;
			s_e.setFactor(0, x+1);
			State s_s = s;
			s_s.setFactor(1, y+1);
			State s_w = s;
			s_w.setFactor(0, x-1);
			
			for (int i=0; i<4; i++) {
				Action a(_domain);
				a.setFactor(0, i);
				mdp->setR(s, a, _config->getRewardStep());
				State s_f = s;
				State s_l = s;
				State s_r = s;
				switch (i) {
				case 0:
					if (!getWallNorth(x, y)) s_f = s_n;
					if (!getWallEast(x, y)) s_r = s_e;
					if (!getWallWest(x, y)) s_l = s_w;
					break;
				case 1:
					if (!getWallEast(x, y)) s_f = s_e;
					if (!getWallSouth(x, y)) s_r = s_s;
					if (!getWallNorth(x, y)) s_l = s_n;
					break;
				case 2:
					if (!getWallSouth(x, y)) s_f = s_s;
					if (!getWallWest(x, y)) s_r = s_w;
					if (!getWallEast(x, y)) s_l = s_e;
					break;
				case 3:
					if (!getWallWest(x, y)) s_f = s_w;
					if (!getWallNorth(x, y)) s_r = s_n;
					if (!getWallSouth(x, y)) s_l = s_s;
					break;	
				}
				Probability p = 0;
				mdp->setT(s, a, s_f, p+_config->getSlipForward());
				p = mdp->T(s, a)->P(s_r);
				mdp->setT(s, a, s_r, p+_config->getSlipRight());
				p = mdp->T(s, a)->P(s_l);
				mdp->setT(s, a, s_l, p+_config->getSlipLeft());
			}
		}
	return mdp;	
}

crl::State _SlipMaze::getTileState(char c) {
	for (size_t x=0; x<getWidth(); x++)
		for (size_t y=0; y<getHeight(); y++) {
			if (getTile(x, y) != c)
				continue;
			State s(_domain);
			s.setFactor(0, x);
			s.setFactor(1, y);
			s.setFactor(2, 0);
			return s;
		}
	return State();
}

crl::State _SlipMaze::getState(size_t x, size_t y) {
	State s(_domain);
	s.setFactor(0, x);
	s.setFactor(1, y);
	s.setFactor(2, 0);
	return s;
}

StateSet _SlipMaze::getTerminalStates() {
	StateSet ss(new _StateSet());
	for (size_t x=0; x<getWidth(); x++)
		for (size_t y=0; y<getHeight(); y++) {
			if (getTile(x, y) == '#' || getTile(x, y) == 'G')
				ss->insert(getState(x, y));	
		}
		
	
	return ss;	
}

Maze crl::readMaze(istream& is) {
	XMLObject xobj;
	is >> xobj;
	if (xobj.getName() != "Maze")
		throw xml_exception("Expected <Maze>");
	size_t width = atoi(xobj("width").c_str());
	size_t height = atoi(xobj("height").c_str());
	Maze maze(new _Maze(width, height));
	
	
	string v_str = xobj["verticalWalls"].getText();
	istringstream v_is(v_str.c_str());
	for (size_t y=0; y<height; y++) {
		for (size_t x=0; x<width-1; x++) {
			char c;
			v_is >> c;
			maze->setWallEast(x, y, c=='|');
		}
	}
	
	string h_str = xobj["horizontalWalls"].getText();
	istringstream h_is(h_str.c_str());
	for (size_t y=0; y<height-1; y++) {
		for (size_t x=0; x<width; x++) {
			char c;
			h_is >> c;
			maze->setWallSouth(x, y, c=='-');
		}
	}
	
	string tile_str = xobj["tiles"].getText();
	istringstream tile_is(tile_str.c_str());
	for (size_t y=0; y<height; y++) {
		for (size_t x=0; x<width; x++) {
			char c;
			tile_is >> c;
			maze->setTile(x, y, c);
			if (c == 'X') {
				maze->setWallNorth(x, y, true);
				maze->setWallEast(x, y, true);
				maze->setWallSouth(x, y, true);
				maze->setWallWest(x, y, true);
			}
		}
	}
	
	return maze;
}

SlipConfig crl::readSlipConfig(std::istream& is) {
	SlipConfig cfg(new _SlipConfig());
	
	XMLObject xobj;
	is >> xobj;
	if (xobj.getName() != "SlipConfig")
		throw xml_exception("Expected <SlipConfig>");
	XMLObject slipObj = xobj["Slip"];
	cfg->setSlipForward(atof(slipObj["forward"].getText().c_str()));
	cfg->setSlipLeft(atof(slipObj["left"].getText().c_str()));
	cfg->setSlipRight(atof(slipObj["right"].getText().c_str()));
	XMLObject rewardObj = xobj["Reward"];
	
	cfg->setRewardGoal(atof(rewardObj["goal"].getText().c_str()));
	cfg->setRewardPit(atof(rewardObj["pit"].getText().c_str()));
	cfg->setRewardStep(atof(rewardObj["step"].getText().c_str()));
	
	return cfg;
}

