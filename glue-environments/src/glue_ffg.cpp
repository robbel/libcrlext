/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <iostream>
#include <fstream>
#include <cstring>
#include <cassert>
#include <cpputil.hpp>
#include <rlgnmenv.h>
#include <rlglue/Environment_common.h>
#include "crl/env_ffg.hpp"
#include <strxml.hpp>

using namespace std;
using namespace crl;
using namespace cpputil;

//FIXME: is it ok to have one global ffg or does an episode reset require a fresh one to be constructed?
FireFightingGraph _ffg;

namespace crl {

//
// Making the environment available in rl-glue
//

Domain getCRLEnvironmentDomain() {
    assert(_ffg != nullptr);
    return _ffg->getDomain();
}

Environment getCRLEnvironment(Domain domain) {
    assert(_ffg != nullptr);
    return _ffg; // Note: no new copy is constructed
}

} // namespace crl

char paramBuf[256];
const char* env_message(const char* inMessage) {
	if (!strcmp(inMessage, "id"))
		return (char*)"firefighting-graph";
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
            cerr << "Usage: " << argv[0] << " <config>" << endl;
            return EXIT_FAILURE;
    }

    try {
      ifstream is(argv[1]);
      if(!(_ffg = readFFG(is))) {
          cerr << "Input file opening failed: " << argv[1] << endl;
          return EXIT_FAILURE;
      }

      sprintf(paramBuf, "ffg=%s", argv[1]); // todo: print agent layout and initial state
      //paramBuf[0] = '\0'; // the empty string

      // test: obtain FactoredMDP
      time_t start_time = time_in_milli();
      FactoredMDP fmdp = _ffg->getFactoredMDP();
      time_t end_time = time_in_milli();
      cout << "[DEBUG]: exported to FactoredMDP in " << end_time - start_time << "ms." << endl;
#if 0
    Domain domain = fmdp->getDomain();
    for (Size state_index=0; state_index<domain->getNumStates(); state_index++) {
            State s(domain, state_index);
            for (Size action_index=0; action_index<domain->getNumActions(); action_index++) {
                    Action a(domain, action_index);
                    for (Size n_index=0; n_index<domain->getNumStates(); n_index++) {
                            State n(domain, n_index);
                            std::cout << "T(s,a,n) = T(" << s << "," << a << "," << n << "): "
                                      << fmdp->T(s,a,n) << std::endl;
                    }
                    std::cout << "R: " << fmdp->R(s,a) << std::endl;
            }
    }
#endif
    } catch(const cpputil::Exception& e) {
        cerr << e << endl;
        return EXIT_FAILURE;
    }

    // run main glue environment loop
    glue_main_env(0, 0);

    return EXIT_SUCCESS;
}
