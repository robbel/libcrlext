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
    along with mazes.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <fstream>
#include <rlglue/Environment_common.h>
#include <rlgnmenv.h>
#include <cpputil.hpp>
#include <crl/crl.hpp>
#include <crl/flat_tables.hpp>
#include <crl/environment.hpp>
#include "crl/glue_env.hpp"
#include "crl/mazes.hpp"

using namespace std;
using namespace cpputil;
using namespace crl;

SlipMaze _slip_maze;
FlagMaze _flag_maze;

Domain crl::getCRLEnvironmentDomain() {
	if (_slip_maze)
		return _slip_maze->getDomain();
	else
		return _flag_maze->getDomain();
}

Environment crl::getCRLEnvironment(Domain domain) {
	if (_slip_maze) {
		MDP mdp = _slip_maze->getMDP();
		State start_state = _slip_maze->getTileState('S');
		MDPEnvironment env(new _MDPEnvironment(mdp, start_state));
		return env;
	}
	else {
		MDP mdp(new _FlagMDP(_flag_maze));
		State start_state = _flag_maze->getInitialState();
		MDPEnvironment env(new _MDPEnvironment(mdp, start_state));
		return env;
	}
}

char paramBuf[256];
const char* env_message(const char* inMessage) {
	if (!strcmp(inMessage, "id"))
		 return (char*)"maze";
	if (!strcmp(inMessage, "version"))
		return (char*)"1";
	if (!strcmp(inMessage, "param"))
		return paramBuf;
	if (!strncmp(inMessage, "seed", 4)) {
		long seed = atoi(inMessage+5);
		srand(seed);
	}
	return (char*)"";
}

int main(int argc, char** argv) {
	srand(time(0));
	try {
		const char* maze_path;
		const char* config_path;

		if (argc != 3 && argc != 4) {
			cout << "Usage: " << argv[0] << " <maze_path> <config_path> [host:port]" << endl;
			return 1;
		}
		char* host = 0;
		short port = 0;
		if (argc == 5) {
			host = strtok(argv[3], ":");
			port = atoi(strtok(0, ":"));
		}

		maze_path = argv[1];
		config_path = argv[2];

		ifstream in_maze(maze_path);
		Maze m = readMaze(in_maze);
		ifstream in_cfg(config_path);
		SlipConfig cfg = readSlipConfig(in_cfg);

		bool is_slip_maze = true;
		for (size_t x=0; is_slip_maze && x<m->getWidth(); x++)
			for (size_t y=0; is_slip_maze && y<m->getHeight(); y++) {
				if (m->getTile(x, y) == 'F') {
					is_slip_maze = false;
				}
			}

		if (is_slip_maze)
			_slip_maze = SlipMaze(new _SlipMaze(m, cfg));
		else
			_flag_maze = FlagMaze(new _FlagMaze(m, cfg));



		sprintf(paramBuf, "maze=%s sf=%f sl=%f sr=%f rg=%f rp=%f rs=%f",
		        maze_path, cfg->getSlipForward(), cfg->getSlipLeft(),
		        cfg->getSlipRight(), cfg->getRewardGoal(), cfg->getRewardPit(),
		        cfg->getRewardStep());

		glue_main_env(host, port);
		return 0;
	}
	catch (const cpputil::Exception& e) {
		cerr << e << endl;
		return 1;
	}
}

