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
#include <rlgnmenv.h>
#include <rlglue/Environment_common.h>
#include "crl/env_graphprop.hpp"
#include "crl/glue_bigenv.hpp"

using namespace std;
using namespace crl;
using namespace crl::graphprop;
using namespace cpputil;

//
// GLUE network wrapper around GraphProp problem
//

//FIXME: is it ok to have one global ffg or does an episode reset require a fresh one to be constructed?
GraphProp _graphprop;

namespace crl {

//
// Making the environment available in rl-glue
//

Domain getCRLEnvironmentDomain() {
    assert(_graphprop != nullptr);
    return _graphprop->getDomain();
}

BigEnvironment getCRLBigEnvironment(Domain domain) {
    assert(_graphprop != nullptr);
    return _graphprop; // Note: no new copy is constructed
}

} // namespace crl

char paramBuf[256];
const char* env_message(const char* inMessage) {
	if (!strcmp(inMessage, "id"))
		return (char*)"graphprop";
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
  if (argc != 3 && argc != 4) {
          LOG_ERROR("Usage: " << argv[0] << " <config.xml> <graph.dat> [--enable-stdout]");
          return EXIT_FAILURE;
  }

  try {
    ifstream iscfg(argv[1]);
    ifstream isdat(argv[2]);
    if(!(_graphprop = readGraphProp(iscfg, isdat))) {
      LOG_ERROR("Error while reading from " << argv[1] << " or " << argv[2]);
      return EXIT_FAILURE;
    }

    if(argc == 4 && std::string(argv[3]) == "--enable-stdout") {
      _graphprop->enableStdout();
    }

    sprintf(paramBuf, "graphprop=%s,%s", argv[1], argv[2]); // todo: print layout and initial state
    //paramBuf[0] = '\0'; // the empty string

  } catch(const cpputil::Exception& e) {
      cerr << e << endl;
      return EXIT_FAILURE;
  }

  // run main glue environment loop
  glue_main_env(0, 0);

  return EXIT_SUCCESS;
}
