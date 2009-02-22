/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth

    This file is part of CRL:RL-Glue.

    CRL:RL-Glue is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CRL:RL-Glue is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CRL:RL-Glue.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <rlglue/Agent_common.h>
#include <rlgnmagent.h>
#include <crl/crl.hpp>
#include <crl/fdomain.hpp>
#include "crl/glue_agent.hpp"

using namespace crl;

Agent crl::getCRLAgent(Domain domain) {
	Learner learner;
	Planner planner(new _RandomPlanner(domain));
	Agent agent(new _Agent(planner, learner));
	return agent;
}

const char* agent_message(const char* inMessage) {
	if (!strcmp(inMessage, "id"))
		return (char*)"random";
	else if (!strcmp(inMessage, "version"))
		return (char*)"1";
	else
		return (char*)"";
}


int main(int argc, char** argv) {
	char* host = 0;
	short port = 0;
	if (argc == 2) {
		host = strtok(argv[1], ":");
		port = atoi(strtok(0, ":"));
	}
	glue_main_agent(host, port);

	return 0;
}
