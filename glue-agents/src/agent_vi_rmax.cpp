/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth

    This file is part of CRL:RL-Glue:agents.

    CRL:RL-Glue:agents is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CRL:RL-Glue:agents is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CRL:RL-Glue:agents.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <rlglue/Agent_common.h>
#include <crl/crl.hpp>
#include <crl/rmax.hpp>
#include <crl/vi.hpp>
#include <crl/flat_tables.hpp>
#include <rlgnmagent.h>
#include "crl/glue_agent.hpp"

using namespace std;
using namespace crl;

int _m;
float _gamma;
Reward _epsilon;

Agent crl::getCRLAgent(Domain domain) {
	FMDPLearner mdp_learner(new _FMDPLearner(domain));
	KnownClassifier classifier(new _FKnownClassifier(domain, _m));
	ActionIterator itr(new _ActionIncrementIterator(domain));

	Reward vmax = domain->getRewardRange().getMax();
	if (_gamma < 1)
		vmax = domain->getRewardRange().getMax()/(1-_gamma);

	RMaxMDPLearner rmaxLearner(new _RMaxMDPLearner(mdp_learner, classifier, itr, vmax));
	VIPlanner planner(new _FlatVIPlanner(domain, rmaxLearner, _epsilon, _gamma));
	VIPlannerAgent agent(new _VIPlannerAgent(planner, rmaxLearner));
	return agent;
}

StateMapper crl::getStateMapper() {
	return StateMapper();
}

char params[256];
const char* agent_message(const char* inMessage) {
	if (strcmp(inMessage,"id")==0)
		return (char*)"vi_rmax";
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


int main(int argc, char** argv) {
	srand(time(0));

	if (argc != 4 && argc != 5) {
		cerr << "Usage: " << argv[0] << " <m> <gamma> <epsilon> [host:port]" << endl;
		return 1;
	}
	_m = atoi(argv[1]);
	_gamma = atof(argv[2]);
	_epsilon = atof(argv[3]);
	char* host = 0;
	short port = 0;
	if (argc == 5) {
		host = strtok(argv[4], ":");
		port = atoi(strtok(0, ":"));
	}

	sprintf(params, "m=%d gamma=%f epsilon=%f", _m, _gamma, _epsilon);

	glue_main_agent(host, port);

	return 0;
}
