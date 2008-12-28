/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth

    This file is part of CRL:RL-Glue:agents.

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

#include <unistd.h>
#include <iostream>
#include <fstream>
#include <rlglue/Agent_common.h>
#include <crl/crl.hpp>
#include <crl/rmax.hpp>
#include <crl/vi.hpp>
#include <crl/uct.hpp>
#include <crl/ps.hpp>
#include <crl/rtdp.hpp>
#include <crl/fdomain.hpp>
#include <crl/mazes.hpp>
#include <rlgnmagent.h>
#include "crl/glue_agent.hpp"

using namespace std;
using namespace crl;

vector<string> args;
char params[256];
MDP mdp;
Domain domain;
StateSet terminal_states;

Agent crl::getCRLAgent(Domain domain) {
	Planner planner;
	if (args[0] == "vi") {
		if (args.size() != 3) {
			cerr << "vi parameters: <gamma> <epsilon>" << endl;
			exit(1);
		}
		float gamma = atof(args[1].c_str());
		Reward epsilon = atof(args[2].c_str());
		sprintf(params, "planner=vi gamma=%f epsilon=%f", gamma, epsilon);
		VIPlanner vi_planner = VIPlanner(new _FactoredVIPlanner(domain, mdp, epsilon, gamma));

		vi_planner->plan();
//		cerr << "planned in " << c << " iterations" << endl;
		planner = vi_planner;
	}

	if (args[0] == "ps") {
		if (args.size() != 3) {
			cerr << "ps parameters: <gamma> <epsilon>" << endl;
			exit(1);
		}
		float gamma = atof(args[1].c_str());
		Reward epsilon = atof(args[2].c_str());
		sprintf(params, "planner=ps gamma=%f epsilon=%f", gamma, epsilon);
		PSPlanner ps_planner = PSPlanner(new _FlatPSPlanner(domain, mdp, epsilon, gamma));
		StateIterator sitr = StateIterator(new _SharedStateSetIterator(terminal_states));
		ps_planner->insert(sitr);
		ps_planner->sweep();

//		ps_planner->plan();
//		cerr << "planned in " << c << " updates" << endl;
		planner = ps_planner;
	}
	if (args[0] == "rtdp") {
		if (args.size() != 7) {
			cerr << "rtdp parameters: <gamma> <epsilon> <m> <runlimit> <timelimit> <heuristic>" << endl;
			exit(1);
		}
		float gamma = atof(args[1].c_str());
		Reward epsilon = atof(args[2].c_str());
		int m = atoi(args[3].c_str());
		int run_limit = atoi(args[4].c_str());
		int time_limit = atoi(args[5].c_str());
		int h = atoi(args[6].c_str());
		sprintf(params, "planner=rtdp gamma=%f epsilon=%f m=%d runlimit=%d timelimit=%d", gamma, epsilon, m, run_limit, time_limit);

		RTDPPlanner rtdp_planner(new _FlatRTDPPlanner(domain, mdp, gamma, epsilon, m, h, 50));
		rtdp_planner->setRunLimit(run_limit);
		rtdp_planner->setTimeLimit(time_limit);
		planner = rtdp_planner;
	}

	if (args[0] == "uct") {
		if (args.size() != 8) {
			cerr << "uct parameters: <gamma> <reward_type> <clear_tree> <full_tree> <run_limit> <time_limit> <C>" << endl;
			exit(1);
		}
		float gamma = atof(args[1].c_str());
		int reward_type = atoi(args[2].c_str());
		bool clear_tree = args[3].compare("true") == 0 ? true : false;
		bool full_tree = args[4].compare("true") == 0 ? true : false;
		int run_limit = atoi(args[5].c_str());
		int time_limit = atoi(args[6].c_str());
		float C = atof(args[7].c_str());
		sprintf(params, "planner=uct gamma=%f reward_type=%d clear_tree=%d full_tree=%d runlimit=%d timelimit=%d C=%f", gamma, reward_type, clear_tree, full_tree, run_limit, time_limit, C);
		cerr << "Params given to the planner = " << params << endl;
		UCTPlanner uct_planner = UCTPlanner(new _FlatUCTPlanner(domain, mdp, gamma, reward_type, clear_tree, full_tree, C));
		uct_planner->setTimeLimit(time_limit);
		uct_planner->setRunLimit(run_limit);
		planner = uct_planner;
	}

	if (args[0] == "mb-uct") {
		if (args.size() != 8) {
			cerr << "mb-uct parameters: <gamma> <reward_type> <clear_tree> <full_tree> <run_limit> <time_limit> <C>" << endl;
			exit(1);
		}
		float gamma = atof(args[1].c_str());
		int reward_type = atoi(args[2].c_str());
		bool clear_tree = args[3].compare("true") == 0 ? true : false;
		bool full_tree = args[4].compare("true") == 0 ? true : false;
		int run_limit = atoi(args[5].c_str());
		int time_limit = atoi(args[6].c_str());
		float C = atof(args[7].c_str());
		sprintf(params, "planner=uct gamma=%f reward_type=%d clear_tree=%d full_tree=%d runlimit=%d timelimit=%d C=%f", gamma, reward_type, clear_tree, full_tree, run_limit, time_limit, C);
		cerr << "Params given to the planner = " << params << endl;
		MBUCTPlanner uct_planner = MBUCTPlanner(new _FlatMBUCTPlanner(domain, mdp, gamma, reward_type, clear_tree, full_tree, C));
		uct_planner->setTimeLimit(time_limit);
		uct_planner->setRunLimit(run_limit);
		planner = uct_planner;
	}

	Agent agent(new _Agent(planner));
	return agent;
}

const char* agent_message(const char* inMessage) {
	if (strcmp(inMessage,"id")==0)
		return (char*)"expert_maze";
	if (strcmp(inMessage,"version")==0)
		return (char*)"1";
	if (strcmp(inMessage,"param")==0)
		return params;
	if (!strncmp(inMessage, "seed", 4)) {
		long seed = atoi(inMessage+5);
		srand(seed);
	}
	return (char*)"";
}

void usage(char** argv) {
	cerr << "Usage: " << argv[0] << " <maze path> <config path> <agent type> [agent parameters]" << endl;
}

int main(int argc, char** argv) {
	srand(time(0));

	if (argc <4) {
		usage(argv);
		return 1;
	}
	const char* maze_path = argv[1];
	const char* config_path = argv[2];

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

	if (is_slip_maze) {
		SlipMaze _slip_maze = SlipMaze(new _SlipMaze(m, cfg));
		mdp = _slip_maze->getMDP();
		domain = _slip_maze->getDomain();
		terminal_states = _slip_maze->getTerminalStates();
	}
	else {
		FlagMaze _flag_maze = FlagMaze(new _FlagMaze(m, cfg));
		mdp = MDP(new _FlagMDP(_flag_maze));
		domain = _flag_maze->getDomain();
		terminal_states = _flag_maze->getTerminalStates();
		mdp->predecessors(*(terminal_states->begin()));
	}
	//mdp->printXML(cerr);


	for (int i=3; i<argc; i++)
		args.push_back(argv[i]);


	try {
		glue_main_agent();
	}
	catch (cpputil::Exception e) {
		cerr << e << endl;
	}

	return 0;
}
