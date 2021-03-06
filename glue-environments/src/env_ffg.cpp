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
#include "crl/env_ffg.hpp"
#include <strxml.hpp>

using namespace std;
using namespace crl;
using namespace cpputil;

namespace crl {

_FireFightingGraph::_FireFightingGraph(Domain domain)
: _domain(std::move(domain)) {

    _num_houses = _domain->getNumStateFactors();
    _num_agents = _domain->getNumActionFactors();
    // some basic parameter checks..
    if(_domain->isBig() ||
       _num_houses <= 0 ||
       _num_agents <= 0 ||
       !in_pos_interval(_num_agents, _num_houses)) {
      throw InvalidException("House or agent number invalid in supplied domain.");
    }
    // obtain the number of fire levels in this problem
    const RangeVec& ranges = _domain->getStateRanges();
    _num_fls = ranges[0].getSpan()+1;
}

Size _FireFightingGraph::getNumAgentsAtHouse(const Action& a, Size h, bool joint_action) const {
    assert(!_house_map.empty() && in_pos_interval(h, _num_houses));
#if 0
    // linear search over agents
    Size num = 0;
    for(Size i = 0; i < _num_agents; i++) {
      if(_agent_locs[i] == h-1)
        num += a.getFactor(i); // either 0 (for left) or 1 (for right)
      else if(_agent_locs[i] == h)
        num += 1-a.getFactor(i);
    }
    return num;
#endif
    // consider only those agents in scope-of-influence of house h
    Size num = 0;
    Size agentNo = 0;
    const auto& ret = _house_map.equal_range(h); // the agent ids co-located with h
    for(auto it = ret.first; it!=ret.second; ++it) { // over all agents in scope
        if(_agent_locs[it->second] == h-1)
           num += a.getFactor(joint_action ? it->second : agentNo); // either 0 (for left) or 1 (for right)
        else if(_agent_locs[it->second] == h)
           num += 1-a.getFactor(joint_action ? it->second : agentNo);
        agentNo++;
    }
    return num;
}

Reward _FireFightingGraph::getReward(const State& n) const {
  Reward r = 0.;
  for(Size i = 0; i < n.size(); i++) {
    r -= n.getFactor(i);
  }
  return r;
}

Domain _FireFightingGraph::getDomain() const {
    return _domain;
}

FactoredMDP _FireFightingGraph::getFactoredMDP() const {
  assert(!_agent_locs.empty());
  FactoredMDP fmdp = boost::make_shared<_FactoredMDP>(_domain);
  vector<Factor> fls(_num_fls-1);
  std::iota(fls.begin(), fls.end(), 1);

  time_t start_time = time_in_milli();
  const RangeVec& ranges = _domain->getStateRanges();
  for(Size h = 0; h < _num_houses; h++) {
      // create a dbn factor
      DBNFactor fa = boost::make_shared<_DBNFactor>(_domain, h);
      LRF lrf = boost::make_shared<_LRF>(_domain); // those have equivalent scopes here
      Size h_local = 0;
      if(h > 0) {
        fa->addDelayedDependency(h-1);
        lrf->addStateFactor(h-1);
        h_local++;
      }
      fa->addDelayedDependency(h); // dependence on self
      lrf->addStateFactor(h);
      if(h < _num_houses-1) {
        fa->addDelayedDependency(h+1);
        lrf->addStateFactor(h+1);
      }
      for(Size i = 0; i < _num_agents; i++) {
          if(_agent_locs[i] == h-1 || _agent_locs[i] == h) {
            fa->addActionDependency(i);
            lrf->addActionFactor(i);
          }
      }
      fa->pack();
      lrf->pack();

      // fill in transition function
      Domain subdomain = fa->getSubdomain();
      for (Size state_index=0; state_index<subdomain->getNumStates(); state_index++) {
              State s(subdomain, state_index);
              for (Size action_index=0; action_index<subdomain->getNumActions(); action_index++) {
                      Action a(subdomain, action_index); // Note: in DBN factor scope, at most two agents
                      for(Factor f=0; f<=ranges[h].getSpan(); f++) {
                          // collect some domain specific features
                          const Factor cur_level    = s.getFactor(h_local); // current fire level
                          const Factor same_level   = cur_level;
                          const Factor higher_level = min(cur_level+1, _num_fls-1);
                          const Factor lower_level  = (cur_level==0) ? 0 : (cur_level-1);
                          const Factor next_level   = ranges[h].getMin()+f; // postulated next fire level
                          const Size agents_at_h    = getNumAgentsAtHouse(a, h, false); // agents fighting fire at h
                          bool burning_neighbor     = false;
                          if((h > 0 && s.getFactor(h_local-1) > 0) || // left burning
                             (h < _num_houses-1 && s.getFactor(h_local+1) > 0)) { // right burning
                              burning_neighbor = true;
                          }

                          // determine transition probabilities
                          Probability p = 0.; // Probability p(f|<scope_h>)
                          switch(agents_at_h) {
                            case(0):
                              //this is kind of strange: when a house is not burning, but
                              //its neigbhor is, it will increase its FL with p=0.8
                              //but when it is already burning (and its neighbor is not), it
                              //increase with p=0.4...

                              //fire is likely to increase
                              if(burning_neighbor) {
                                  if(next_level == same_level)
                                    p+=0.2;
                                  if(next_level == higher_level)
                                    p+=0.8;
                              }
                              else if (cur_level == 0) { //fire won't get ignited
                                  if(0 == next_level)
                                    p=1.0;
                                  else //not possible so we can quit...
                                    p=0.0;
                              }
                              else { //normal burning house
                                  if(next_level == same_level)
                                    p+=0.6;
                                  if(next_level == higher_level)
                                    p+=0.4;
                              }
                              break;
                          case(1):
                              //fire is likely to decrease
                              if(burning_neighbor) {
                                  if(next_level == same_level)
                                    p+=0.4;
                                  if(next_level == lower_level)
                                    p+=0.6; //.6 prob of extuinguishing 1 fl
                              }
                              else if (cur_level == 0) { //fire won't get ignited
                                  if(0 == next_level)
                                    p=1.0;
                                  else //not possible so we can quit...
                                    p=0.0;
                              }
                              else { //normal burning house
                                  if(next_level == same_level)
                                    p+=0.0;
                                  if(next_level == lower_level)
                                    p+=1.0;
                              }
                              break;
                          default:
                              //more than 1 agent: fire is extinguished
                              if(0 == next_level)
                                p=1.0;
                              else //not possible so we can quit...
                                p=0.0;
                          }
                          // update probability
                          fa->setT(s,a,ranges[h].getMin()+f,p);
                      }

                      // set local reward function for hourse h
                      const ProbabilityVec& probs = fa->T(s,a);
                      Reward r = -std::inner_product(probs.begin()+1, probs.end(), fls.begin(), static_cast<Probability>(0.));
                      lrf->setR(s,a,r);
              }
      }

      // add random factor to dbn
      fmdp->addDBNFactor(std::move(fa));
      fmdp->addLRF(std::move(lrf));
      //cout << "[DEBUG]: added DBNFactor for house " << h << endl;
  }
  time_t end_time = time_in_milli();
  cout << "[DEBUG]: created (factored) transitions and rewards in " << end_time - start_time << "ms." << endl;
#if 0
  start_time = time_in_milli();
  // compute (flat) reward function
  const DBN& dbn = fmdp->T();
  const FactorIterator& fitr = dbn->factors();
  vector<Factor> fls(_num_fls-1);
  std::iota(fls.begin(), fls.end(), 1);
  for (Size state_index=0; state_index<_domain->getNumStates(); state_index++) {
          State s(_domain, state_index);
          for (Size action_index=0; action_index<_domain->getNumActions(); action_index++) {
                  Action a(_domain, action_index);
                  //cout << "s: " << s << " a: " << a << endl;
                  Reward r = 0;
                  fitr->reset();
                  while(fitr->hasNext()) {
                      const DBNFactor& sf = fitr->next();
                      const ProbabilityVec& probs = sf->T(s,a);
                      assert(probs.size() == static_cast<unsigned>(_num_fls));
                      r -= std::inner_product(probs.begin()+1, probs.end(), fls.begin(), static_cast<Probability>(0.));
                  }
                  fmdp->setR(s,a,r);
          }
  }
  end_time = time_in_milli();
  cout << "[DEBUG]: created (flat) reward function in " << end_time - start_time << "ms." << endl;
#endif

  return fmdp;
}

void _FireFightingGraph::setAgentLocs(std::string locs) {
    _agent_locs.clear();
    _house_map.clear();

    if(locs.empty()) {
        //TODO: randomize assignment
        cerr << "Location randomization not implemented yet" << endl;
    }

    std::stringstream stream(locs);
    string tok;
    while(std::getline(stream, tok, ',')) {
        Size loc = std::stoull(tok);
        if(in_pos_interval(loc, _num_houses-1)) {
            _house_map.emplace(loc, _agent_locs.size());
            _house_map.emplace(loc+1, _agent_locs.size());
            _agent_locs.push_back(loc);
        } else {
            throw InvalidException("Invalid agent location in location string.");
        }
    }
    if(_agent_locs.size() != _num_agents) {
      throw InvalidException("Agent number is different from those in location string.");
    }
}

State _FireFightingGraph::begin() {
    _current = State(_domain);
    for(Size h = 0; h < _num_houses; h++) {
        _current.setFactor(h, static_cast<Factor>(randDouble()*_num_fls));
    }
    return _current;
}

bool _FireFightingGraph::isTerminated() {
  // this flattened index should correspond to the all-zeroes state
  return _current.getIndex() == 0;
}

// apply action, obtain resulting state n, update _current
Observation _FireFightingGraph::getObservation(const Action& a) {

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
        domain->addStateFactor(0, num_fls-1, "house_"+to_string(i));
    }
    // two types of actions: '0' for left, '1' for right
    for(int i = 0; i < num_agents; i++) {
        domain->addActionFactor(0, 1, "agent_"+to_string(i));
    }
    domain->setRewardRange(-num_fls+1, 0.); // FIXME: over all houses!

    // instantiate ffg problem
    FireFightingGraph ffg = boost::make_shared<_FireFightingGraph>(std::move(domain));
    ffg->setAgentLocs(agentAssignments);
    return ffg;
}

} // namespace crl
