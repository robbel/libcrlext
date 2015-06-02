/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <iostream>
#include <cassert>
#include <string.h>
#include <rlglue/Agent_common.h>
#include <rlgnmagent.h>
#include "crl/crl.hpp"
#include "crl/spudd.hpp"
#include "crl/glue_agent.hpp"
#include "cpputil.hpp"
#include "logger.hpp"

using namespace crl;

const char * _policy_file;

namespace crl {

//
// Making the agent available in rl-glue
//

Agent getCRLAgent(Domain domain) {
    assert(_policy_file != nullptr);
    Agent agent;
    try {
        Learner learner;
        Policy policy(new _SpuddPolicy(domain, _policy_file));
        agent = boost::make_shared<_Agent>(policy, learner);
    }
    catch(const cpputil::Exception& e) {
        LOG_FATAL(e);
        throw;
    }

    return agent;
}

StateMapper getStateMapper() {
    return nullptr;
}

} // namespace crl

char params[256];
const char* agent_message(const char* inMessage) {
	if (strcmp(inMessage,"id")==0)
		return (char*)"spudd";
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

// launch networked rl-glue agent through rlgnm library
int main(int argc, char** argv) {
    if (argc != 2) {
            LOG_ERROR("Usage: " << argv[0] << " <SPUDD-OPTDual.ADD file>");
            return EXIT_FAILURE;
    }

    _policy_file = argv[1];

    sprintf(params, "policy=%s", argv[1]);
    //params[0] = '\0'; // the empty string

#if 0
	char* host = 0;
	short port = 0;
	if (argc == 3) {
		host = strtok(argv[2], ":");
		port = atoi(strtok(0, ":"));
	}
#endif
	// run main glue agent loop
	glue_main_agent(0, 0);

	return EXIT_SUCCESS;
}
