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
#include <cpputil.hpp>
#include "crl/env_ffg.hpp"

using namespace std;
using namespace crl;


namespace crl {

//
// TODO: implement
//

/*
  Some ideas:
    ctor with nrHouses, nrAgents, nrFirelevels
      Note: in domain, give variables meaningful names via string
    distribute agents randomly (?) among houses
    maybe include lrf -- just another reward_domain (?)

    while(true) draw without replacement a loc and assign next agent
    maybe support direct assignment in config file (which agent which slot)

 */

_FireFightingGraph::_FireFightingGraph(Domain domain)
: _domain(std::move(domain)) { }

Size _FireFightingGraph::getNumAgentsAtHouse(const Action& a, Size h) {
  return 0;
}


Reward _FireFightingGraph::getReward(const State& n) const
{
  Reward r = 0.;
  for(Size i = 0; i < n.size(); i++) {
    r -= n.getFactor(i);
  }
  return r;
}

State _FireFightingGraph::begin() {

//  _current = State(_domain); // really?
//  _current.setFactor(0, 0); // and what does that do different from initialization above?
//  return _current;

  return State();
}

bool _FireFightingGraph::isTerminated() {
  // this flattened index should correspond to the all-zeroes state
  return _current.getIndex() == 0;
}

Observation _FireFightingGraph::getObservation(const Action& a) {
  // apply action, obtain resulting state n, map that to _current

  //Probability r = cpputil::randDouble();

  //
  // todo..
  //

  Observation o = boost::make_shared<_Observation>(_current, getReward(_current));
  return o;
}

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
		return paramBuf; // return empty string
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

  paramBuf[0] = '\0'; // the empty string

  glue_main_env(0, 0);

  return EXIT_SUCCESS;
}
