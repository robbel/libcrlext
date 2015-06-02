/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <string.h>
#include <rlglue/Agent_common.h>
#include <rlgnmagent.h>
#include <crl/crl.hpp>
#include "crl/glue_agent.hpp"

using namespace crl;

Agent crl::getCRLAgent(Domain domain) {
	Learner learner;
	Policy policy(new _NullPolicy(domain));
	Agent agent(new _Agent(domain, policy, learner));
	return agent;
}

StateMapper crl::getStateMapper() {
	return nullptr;
}

extern "C" const char* agent_message(const char* inMessage) {
	if (!strcmp(inMessage, "id"))
		return (char*)"null";
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
