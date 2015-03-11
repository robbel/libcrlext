/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <iostream>
#include <rlglue/Environment_common.h>
#include <cstring>
#include <crl/crl.hpp>
#include <rlgnmenv.h>

using namespace std;
using namespace crl;


namespace crl {

//
// TODO: implement
//



//
// Making the environment available in rl-glue
//

Domain getCRLEnvironmentDomain() {
  return nullptr;
}

Environment getCRLEnvironment(Domain domain) {
  return nullptr;
}

}

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
		srand(seed);
	}
	return (char*)"";
}

// launch networked rl-glue environment through rlgnm library
int main(int argc, char** argv) {
  cout << "todo" << endl;


  paramBuf[0] = '\0'; // initialize paramBuf for good measure

  glue_main_env(0, 0);

  return EXIT_SUCCESS;
}
