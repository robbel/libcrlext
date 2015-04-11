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

//FIXME: is it ok to have one global sysadmin or does an episode reset require a fresh one to be constructed?
Sysadmin _sysadmin;

namespace crl {

_Sysadmin::_Sysadmin(Domain domain) {
}

Domain _Sysadmin::getDomain() const {
  return nullptr;
}

FactoredMDP _Sysadmin::getFactoredMDP() const {
  return nullptr;
}

State _Sysadmin::begin() {
  return State();
}

bool _Sysadmin::isTerminated() {
  return false;
}

Observation _Sysadmin::getObservation(const Action& a) {
  return nullptr;
}

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
    if (argc != 2) {
            cerr << "Usage: " << argv[0] << " <computer_number>" << endl;
            return EXIT_FAILURE;
    }

    //_sysadmin = boost::make_shared<_Sysadmin>(std::move(domain));

    sprintf(paramBuf, "sysadmin=%s", argv[1]);
    //paramBuf[0] = '\0'; // the empty string
#if 0
    // test: obtain FactoredMDP
    time_t start_time = time_in_milli();
    FactoredMDP fmdp = _sysadmin->getFactoredMDP();
    time_t end_time = time_in_milli();
    cout << "[DEBUG]: exported to FactoredMDP in " << end_time - start_time << "ms." << endl;
#endif
    // run main glue environment loop
    glue_main_env(0, 0);

    return EXIT_SUCCESS;
}
