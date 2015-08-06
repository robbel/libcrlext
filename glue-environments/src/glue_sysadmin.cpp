/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <iostream>
#include <rlgnmenv.h>
#include <rlglue/Environment_common.h>
#include "crl/env_sysadmin.hpp"

using namespace std;
using namespace crl;
using namespace crl::sysadmin;
using namespace cpputil;

//
// GLUE network wrapper around SysAdmin problem
//

//FIXME: is it ok to have one global sysadmin or does an episode reset require a fresh one to be constructed?
Sysadmin _sysadmin;

namespace crl {

//
// Making the environment available in rl-glue
//

Domain getCRLEnvironmentDomain() {
    assert(_sysadmin != nullptr);
    return _sysadmin->getDomain();
}

Environment getCRLEnvironment(Domain domain) {
    assert(_sysadmin != nullptr);
    return _sysadmin; // Note: no new copy is constructed
}

} // namespace crl

char paramBuf[256];
const char* env_message(const char* inMessage) {
	if (!strcmp(inMessage, "id"))
		return (char*)"sysadmin";
	else if (!strcmp(inMessage, "param"))
		return paramBuf;
	else if (!strcmp(inMessage, "version"))
		return (char*)"1";
	else if (!strncmp(inMessage, "seed", 4)) {
		long seed = atoi(inMessage+5);
		srand(seed); // configure the random seed
	}
	return (char*)"";
}

// launch networked rl-glue environment through rlgnm library
int main(int argc, char** argv) {
    if (argc != 3 && argc != 5) {
            LOG_ERROR("Usage: " << argv[0] << " <\"star\"|\"ring\"> <computer_number> [-t \"simple\"]");
            return EXIT_FAILURE;
    }

    try {
        vector<string> remParams(argv+3, argv+argc);
        bool simple = false;
        auto tIt = std::find(remParams.begin(), remParams.end(), "-t");
        if(tIt != remParams.end() && (++tIt) != remParams.end()) {
            simple = *tIt == "simple";
        }
        long long comp_no = std::atoll(argv[2]);
        if(comp_no <= 0 || !(_sysadmin = !simple ? buildSysadmin(argv[1], static_cast<Size>(comp_no)) :
                             buildSimpleSysadmin(argv[1], static_cast<Size>(comp_no)))) {
            LOG_ERROR("Instantiation of " << (simple ? "simple " : "") << "Multi-agent Sysadmin problem failed.");
            return EXIT_FAILURE;
        }

        sprintf(paramBuf, (!simple ? "ma-sysadmin=%s,%s" : "sysadmin=%s,%s"), argv[1], argv[2]);
        //paramBuf[0] = '\0'; // the empty string

    } catch(const cpputil::Exception& e) {
        LOG_ERROR(e);
        return EXIT_FAILURE;
    }

    // run main glue environment loop
    glue_main_env(0, 0);

    return EXIT_SUCCESS;
}

