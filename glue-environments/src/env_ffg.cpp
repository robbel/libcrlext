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

FireFightingGraph _ffg;

namespace {

/// \brief True iff val in [0,max)
template<class T>
bool in_pos_interval(const T& val, const T& max) {
    return val >= 0 && val < max;
}

} // anonymous namespace

namespace crl {

/*
  TODO
      In domain, give variables meaningful names via string
 */

_FireFightingGraph::_FireFightingGraph(Domain domain)
: _domain(std::move(domain)) {

}

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

Domain _FireFightingGraph::getDomain() const {
    return _domain;
}

void _FireFightingGraph::setAgentLocs(std::string locs) {

    if(locs.empty()) {
        //TODO: randomize assignment
    } else if(locs.length() != _num_agents) {
        throw cpputil::InvalidException("Agent number is different from length of location string.");
    }

    for(auto c : locs) {
        Size loc = c - '0'; // convert char to unsigned
        if(!in_pos_interval(loc, _num_houses-1)) {
            _agent_locs.push_back(loc);
        } else {
            throw cpputil::InvalidException("Invalid agent location in location string.");
        }
    }
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
    assert(_ffg != nullptr);
    return _ffg->getDomain();
}

Environment getCRLEnvironment(Domain domain) {
    assert(_ffg != nullptr);
    return _ffg; // Note: no new copy is constructed
}

FireFightingGraph readFFG(std::istream& is) {
//
// The FireFightingGraph layout:
//    <FFG>
//        <Houses count="5"/>
//        <Agents count="3">013</Agents> <!-- or random if no locs are given -->
//        <FireLevels count="3"/>
//    </FFG>
//
    // read problem layout from xml file
    XMLObject xobj(is);
    XMLObject houses = xobj["Houses"];
    int num_houses = atoi(houses("count").c_str());
    XMLObject agents = xobj["Agents"];
    int num_agents = atoi(agents("count").c_str());
    string agentAssignments = agents.getText();
    XMLObject fls = xobj["FireLevels"];
    int num_fls = atoi(fls("count").c_str());

    // construct domain
    Domain domain = boost::make_shared<_Domain>();
    for(int i = 0; i < num_houses; i++) {
        domain->addStateFactor(0, num_fls, "house_"+i);
    }
    for(int i = 0; i < num_agents; i++) {
        domain->addActionFactor(0, 1, "agent_"+i);
    }
    domain->setRewardRange(-num_fls, 0.);

    // instantiate ffg problem
    FireFightingGraph ffg = boost::make_shared<_FireFightingGraph>(std::move(domain));
    ffg->setAgentLocs(agentAssignments);
    return ffg;
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
    if (argc != 2) {
            cerr << "Usage: " << argv[0] << " <config>" << endl;
            return EXIT_FAILURE;
    }
    ifstream is(argv[1]);
    _ffg = readFFG(is);

    paramBuf[0] = '\0'; // the empty string

    glue_main_env(0, 0);

    return EXIT_SUCCESS;
}
