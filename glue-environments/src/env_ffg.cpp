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

_FireFightingGraph::_FireFightingGraph(Domain domain)
: _domain(std::move(domain)) {

    _num_houses = _domain->getNumStateFactors();
    _num_agents = _domain->getNumActionFactors();
    // some basic parameter checks..
    if(_num_houses <= 0 ||
       _num_agents <= 0 ||
       !in_pos_interval(_num_agents, _num_houses)) {
      throw InvalidException("House or agent number invalid in supplied domain.");
    }
    // obtain the number of fire levels in this problem
    const RangeVec& ranges = _domain->getStateRanges();
    _num_fls = ranges[0].getMax();
}

Size _FireFightingGraph::getNumAgentsAtHouse(const Action& a, Size h) {
    assert(!_agent_locs.empty() && in_pos_interval(h, _num_houses));

    Size num = 0;
    if(h != 0)
      num += a.getFactor(h-1); // either 0 (for left) or 1 (for right);
    if(h < _num_houses-1)
      num += 1-a.getFactor(h);

    return num;
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
    _agent_locs.clear();

    if(locs.empty()) {
        //TODO: randomize assignment
        cerr << "Location randomization not implemented yet" << endl;
    } else if(locs.length() != _num_agents) {
        throw InvalidException("Agent number is different from length of location string.");
    }

    for(auto c : locs) {
        Size loc = c - '0'; // convert char to unsigned
        if(in_pos_interval(loc, _num_houses-1)) {
            _agent_locs.push_back(loc);
        } else {
            throw InvalidException("Invalid agent location in location string.");
        }
    }
}

State _FireFightingGraph::begin() {
    //_current = State(_domain); // really?
    for(Size h = 0; h < _num_houses; h++) {
        _current.setFactor(h, randDouble()*_num_fls);
    }
    return _current;
}

bool _FireFightingGraph::isTerminated() {
  // this flattened index should correspond to the all-zeroes state
  return _current.getIndex() == 0;
}

Observation _FireFightingGraph::getObservation(const Action& a) {
  // apply action, obtain resulting state n, map that to _current
  // FIXME: do i have to care about episode ends here?

  for(Size h = 0; h < _num_houses; h++) {
      // determine burning neighbor
      bool burning_neighbor = false;
      if((h > 0 && _current.getFactor(h-1) > 0) ||
         (h < _num_houses-1 && _current.getFactor(h+1) > 0)) {
          burning_neighbor = true;
      }

      // transition house fire level
      const Factor current_level = _current.getFactor(h);
      const Factor higher_level  = min(current_level+1, _num_fls-1);
      const Factor lower_level   = (current_level==0) ? 0 : (current_level-1);

      Size num_agents = getNumAgentsAtHouse(a, h);
      switch(num_agents) {
          case 0:
              //this is kind of strange: when a house is not burning, but
              //its neigbhor is, it will increase its FL with p=0.8
              //but when it is already burning (and its neighbor is not), it
              //increases with p=0.4...

              //fire is likely to increase
              if(burning_neighbor) {
                  if(randDouble() < 0.2) { }
                  // nextLevel = currentLevel
                  else
                      _current.setFactor(h, higher_level);
              }
              else if (current_level == 0) { //fire won't get ignited
                  // nextLevel = currentLevel
              }
              else { //normal burning house
                  if(randDouble() < 0.6) { }
                  // nextLevel == currentLevel
                  else
                      _current.setFactor(h, higher_level);
              }
              break;
          case 1:
              //fire is likely to decrease
              if(burning_neighbor) {
                  if(randDouble() < 0.4) { }
                  // nextLevel == sameLevel
                  else
                      _current.setFactor(h, lower_level);
              }
              else if (current_level == 0) { //fire won't get ignited
                  // nextLevel = currentLevel
              }
              else { //normal burning house
                  _current.setFactor(h, lower_level);
              }
              break;
          default:
              //more than 1 agent: fire is extinguished
              _current.setFactor(h, 0);
      }
  }

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

//
// The FireFightingGraph layout:
//    <FFG>
//        <Houses count="5"/>
//        <Agents count="3">013</Agents> <!-- Note: location assignment string (e.g., 013) is optional -->
//        <FireLevels count="3"/>
//    </FFG>
//
FireFightingGraph readFFG(std::istream& is) {
    if(!is) {
        return nullptr;
    }

    // read problem layout from xml file
    XMLObject xobj(is);
    if (xobj.getName() != "FFG")
        throw xml_exception("Expected <FFG>");
    XMLObject houses = xobj["Houses"];
    int num_houses = atoi(houses("count").c_str());
    XMLObject agents = xobj["Agents"];
    int num_agents = atoi(agents("count").c_str());
    string agentAssignments = agents.size() != 0 ? agents.getText() : ""; // if an assignment string is given
    XMLObject fls = xobj["FireLevels"];
    int num_fls = atoi(fls("count").c_str());

    // construct domain
    Domain domain = boost::make_shared<_Domain>();
    for(int i = 0; i < num_houses; i++) {
        domain->addStateFactor(0, num_fls-1, "house_"+i);
    }
    // two types of actions: '0' for left, '1' for right
    for(int i = 0; i < num_agents; i++) {
        domain->addActionFactor(0, 1, "agent_"+i);
    }
    domain->setRewardRange(-num_fls+1, 0.);

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

    try {
      ifstream is(argv[1]);
      if(!(_ffg = readFFG(is))) {
          cerr << "Input file opening failed: " << argv[1] << endl;
          return EXIT_FAILURE;
      }
    } catch(const cpputil::Exception& e) {
        cerr << e << endl;
        return EXIT_FAILURE;
    }

    sprintf(paramBuf, "ffg=%s", argv[1]); // todo: print agent layout and initial state
    //paramBuf[0] = '\0'; // the empty string

    // run main glue environment loop
    glue_main_env(0, 0);

    return EXIT_SUCCESS;
}
