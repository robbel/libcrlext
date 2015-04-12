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

//FIXME: is it ok to have one global sysadmin or does an episode reset require a fresh one to be constructed?
Sysadmin _sysadmin;

namespace crl {

_Sysadmin::_Sysadmin(Domain domain, Topology network)
    : _domain(std::move(domain)), _network(network),
      _num_comps(_domain->getNumStateFactors()/2),
      _num_agents(_domain->getNumActionFactors()) {
  // some basic parameter checks
  if(_num_comps == 0 || _num_comps != _num_agents) {
    throw InvalidException("Sysadmin and computer number must match");
  }
  // build factored MDP
  buildFactoredMDP();
}

// Holds for both RING and STAR topologies
void _Sysadmin::buildPlate(Size c, DBNFactor& fas, DBNFactor& fal, LRF& lrf) {
    Domain fasdom = fas->getSubdomain();
    Domain faldom = fal->getSubdomain();

    // Define Status (note variable sorting in local scope!)
    Size t = 1;  // index of this computer in subdomain
    Size n = 0;  // index of neighbor in subdomain
    if(c == 0) { // special case: first computer in network
        if(_network == Topology::RING) {
            t = 0;
            n = 1;
        }
        else {   // in the STAR case, computer 0 (the `server') does not have any neighbors
            t = n = 0;
        }
    }
    _StateActionIncrementIterator saitr(fasdom);
    while(saitr.hasNext()) {
        const std::tuple<State,Action>& sa = saitr.next();
        const State s  = std::get<0>(sa);
        const Action a = std::get<1>(sa);
        if(a.getIndex() == (Size)Admin::REBOOT) {
            fas->setT(s, a, (Factor)Status::GOOD, 1.); // deterministically set to GOOD
        }
        else { // Admin::NOTHING
            const Factor cur = s.getFactor(t); // current status of this computer
            Probability failure = 0.1; // default failure likelihood of this machine
            if(t != n) {
                failure += 0.2 * s.getFactor(n); // increase failure likelihood with broken neighbor
            }
            switch(cur) {
                case (Factor)Status::GOOD:
                    fas->setT(s, a, (Factor)Status::GOOD, 1.0 - failure);
                    fas->setT(s, a, (Factor)Status::FAULTY, failure);
                    break;
                case (Factor)Status::FAULTY:
                    fas->setT(s, a, (Factor)Status::FAULTY, 1.0 - failure);
                    fas->setT(s, a, (Factor)Status::DEAD, failure);
                    break;
                case (Factor)Status::DEAD:
                    fas->setT(s, a, (Factor)Status::DEAD, 1.); // a DEAD machine remains DEAD
                    break;
            }
        }
    }

    // Define Load
    _StateActionIncrementIterator litr(faldom);
    while(litr.hasNext()) {
        const std::tuple<State,Action>& sa = litr.next();
        const State s  = std::get<0>(sa);
        const Action a = std::get<1>(sa);

        if(a.getIndex() == (Size)Admin::REBOOT || s.getFactor(0) == (Factor)Status::DEAD) {
            fal->setT(s, a, (Factor)Load::IDLE, 1.);
        }
        else { // deterministic load switches from IDLE->LOADED->SUCCESS->IDLE->...
            const Factor cur = s.getFactor(1);                 // current load
            const Load next  = static_cast<Load>((cur+1) % 3); // next load after deterministic increment
            fal->setT(s, a, (Factor)next, 1.);
        }
    }

    // Define LRF
    State dummy_s; // we don't really need Domain here
    dummy_s.setIndex(static_cast<Size>(Load::PROCESS_SUCCESS));
    lrf->define(dummy_s, Action(), 1.);
}

void _Sysadmin::buildFactoredMDP() {
    _fmdp = boost::make_shared<_FactoredMDP>(_domain);

    time_t start_time = time_in_milli();
    for(Size i = 0; i < _num_comps; i++) {
        // create dbn factors for computer `i'
        DBNFactor fas = boost::make_shared<_DBNFactor>(_domain, 2*i);   // status
        DBNFactor fal = boost::make_shared<_DBNFactor>(_domain, 2*i+1); // load
        LRF lrf = boost::make_shared<_LRF>(_domain); // those have equivalent scopes here
        // Status variable
        fas->addDelayedDependency(2*i);  // self
        fas->addActionDependency(i);
        if(_network == Topology::RING) { // neighbor
            if(i>0) {
                fas->addDelayedDependency(2*i-2);
            }
            else {
                fas->addDelayedDependency((_num_comps-1)*2); // close the ring
            }
        }
        else { // STAR
            fas->addDelayedDependency(0); // all depend on the first computer (the `server');
        }
        // Load variable
        fal->addDelayedDependency(2*i);   // self status
        fal->addDelayedDependency(2*i+1); // self
        fal->addActionDependency(i);
        // LRF
        lrf->addStateFactor(2*i+1);       // over load variable
        // allocate tabular storage
        fas->pack();
        fal->pack();
        lrf->pack();

        // Fill transition and reward function
        buildPlate(i, fas, fal, lrf);
        // add factors for computer `i' to DBN
        _fmdp->addDBNFactor(std::move(fas));
        _fmdp->addDBNFactor(std::move(fal));
        _fmdp->addLRF(std::move(lrf));
    }

    time_t end_time = time_in_milli();
    cout << "[DEBUG]: created factored MDP in " << end_time - start_time << "ms." << endl;
}

State _Sysadmin::begin() {
  _current = State(_domain);
  for(Size i = 0; i < _num_comps; i+=2) {
      _current.setFactor(i,   (Factor) Status::GOOD);
      _current.setFactor(i+1, (Factor) Load::IDLE);
  }
  return _current;
}

// apply action, obtain resulting state n, update _current
Observation _Sysadmin::getObservation(const Action& a) {
    Reward r = getReward(_current); // reward function is defined over load at current state s

    // base on DBN...


    Observation o = boost::make_shared<_Observation>(_current, r);
    return o;
}

Reward _Sysadmin::getReward(const State& s) const {
  Reward r = 0.;
  // over all Load variables
  for(Size i = 1; i < s.size(); i+=2) {
      // increment for every successful task
      r += static_cast<Reward>(s.getFactor(i) == (Factor) Load::PROCESS_SUCCESS);
  }
  return r;
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

Sysadmin buildSysadmin(string arch, Size num_comps) {
  Domain domain = boost::make_shared<_Domain>();
  // variables
  const vector<Status> status {Status::GOOD,   Status::FAULTY, Status::DEAD};          // 0,1,2
  const vector<Load> load     {Load::IDLE,     Load::LOADED,   Load::PROCESS_SUCCESS}; // 0,1,2
  const vector<Admin> action  {Admin::NOTHING, Admin::REBOOT};                         // 0,1

  for(Size i = 0; i < num_comps; i++) {
      domain->addStateFactor(0, status.size()-1, "status_"+to_string(i));
      domain->addStateFactor(0, load.size()-1,   "load_"+to_string(i));
      domain->addActionFactor(0, action.size()-1);
  }
  domain->setRewardRange(0,1);

  Topology t;
  std::transform(arch.begin(), arch.end(), arch.begin(), ::tolower);
  if(arch == "star") {
      t = Topology::STAR;
  }
  else if(arch == "ring") {
      t = Topology::RING;
  }
  else {
      throw InvalidException("Invalid network topology " + arch);
  }

  // instantiate sysadmin problem
  Sysadmin sysadmin = boost::make_shared<_Sysadmin>(std::move(domain), t);
  return sysadmin;
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
    if (argc != 3) {
            cerr << "Usage: " << argv[0] << " <\"star\"|\"ring\"> <computer_number>" << endl;
            return EXIT_FAILURE;
    }

    if(!(_sysadmin = buildSysadmin(argv[1], std::atoi(argv[2])))) {
        cerr << "Instantiation of Multi-agent Sysadmin problem failed." << endl;
        return EXIT_FAILURE;
    }

    sprintf(paramBuf, "ma-sysadmin=%s,%s", argv[1], argv[2]);
    //paramBuf[0] = '\0'; // the empty string

    // run main glue environment loop
    glue_main_env(0, 0);

    return EXIT_SUCCESS;
}
