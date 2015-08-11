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
#include "crl/env_sysadmin.hpp"
#include "logger.hpp"

using namespace std;
using namespace crl;
using namespace crl::sysadmin;
using namespace cpputil;

namespace crl {

_Sysadmin::_Sysadmin(Domain domain, Topology network)
: _Sysadmin(domain, network, domain->getNumStateFactors()/2, domain->getNumActionFactors()) {
  // build factored MDP
  buildFactoredMDP();
}

_Sysadmin::_Sysadmin(Domain domain, Topology network, Size num_comps, Size num_agents)
: _domain(std::move(domain)), _network(std::move(network)), _num_comps(num_comps), _num_agents(num_agents) {
  // some basic parameter checks
  if(_num_comps == 0 || _num_comps != _num_agents) {
    throw InvalidException("Sysadmin and computer number must match");
  }
}

// Holds for both RING and STAR topologies
void _Sysadmin::buildPlate(Size c, DBNFactor& fas, DBNFactor& fal, LRF& lrf) {
    Domain fasdom = fas->getSubdomain();
    Domain faldom = fal->getSubdomain();

    // Define Status (note variable sorting in local scope!)
    Size t = 1;  // index of this computer in subdomain
    Size n = 0;  // index of neighbor in subdomain
    if(c == 0) { // special case: first computer in network
        if(_network == Topology::RING && _num_comps > 1) {
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
        else { // deterministic load switches from IDLE->LOADED->SUCCESS->IDLE->... unless status is faulty
            const Factor cur = s.getFactor(1);                 // current load
            const Load next  = static_cast<Load>((cur+1) % 3); // next load after deterministic increment
            if(cur == (Factor)Load::LOADED) {
                Probability failure = s.getFactor(0) * 0.2;    // if status is faulty, less likelihood of success
                fal->setT(s, a, (Factor)next, 1.0 - failure);
                fal->setT(s, a, (Factor)cur, failure);
            }
            else {
                fal->setT(s, a, (Factor)next, 1.);             // deterministic progress
            }
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
    LOG_INFO("created factored MDP in " << end_time - start_time << "ms.");
}

State _Sysadmin::begin() {
  _current = State(_domain);
  for(Size i = 0; i < _num_comps*2; i+=2) {
      _current.setFactor(i,   (Factor) Status::GOOD);
      _current.setFactor(i+1, (Factor) Load::IDLE);
  }
  return _current;
}

// apply action, obtain resulting state n, update _current
// TODO make this a generic getObservation() for a `FactoredMDPEnvironment' (\see _MDPEnvironment)
Observation _Sysadmin::getObservation(const Action& ja) {
    Reward r = getReward(_current, ja); // reward function is defined over load at current state s
    // prepare a new state
    State new_current = _current;

    const _DBN& T = _fmdp->T();
    for(int f = 0; f < T.size(); f++) {
        const DBNFactor& fa = T.factor(f);
        const ProbabilityVec& pvec = fa->T(_current, ja);
#if 0
        Probability sum = std::accumulate(pvec.begin(), pvec.end(), 0.);
        assert(approxEq(sum,1.));
#endif
        // determine a new bin (i.e., Factor value) according to transition function
        const Probability r = randDouble();
        Probability acc     = 0;
        Factor bin          = 0;
        do {
            acc += pvec[bin++];
        }
        while(bin < pvec.size() && r > acc);
        assert(r <= acc);

        // update new state
        new_current.setFactor(f, bin-1);
    }

    _current = std::move(new_current);
    Observation o = boost::make_shared<_Observation>(_current, r);
    return o;
}

Reward _Sysadmin::getReward(const State& s, const Action& a) const {
  Reward r = 0.;
  // over all Load variables
  for(Size i = 1; i < s.size(); i+=2) {
      // increment for every successful task
      r += static_cast<Reward>(s.getFactor(i) == (Factor) Load::PROCESS_SUCCESS);
  }
  return r;
}

Sysadmin buildSysadmin(string arch, Size num_comps) {
  Domain domain = boost::make_shared<_Domain>();
  // variables
  const vector<Status> status {Status::GOOD,   Status::FAULTY, Status::DEAD};          // 0,1,2
  const vector<Load>   load   {Load::IDLE,     Load::LOADED,   Load::PROCESS_SUCCESS}; // 0,1,2
  const vector<Admin>  action {Admin::NOTHING, Admin::REBOOT};                         // 0,1

  for(Size i = 0; i < num_comps; i++) {
      domain->addStateFactor(0, status.size()-1, "status_"+to_string(i));
      domain->addStateFactor(0, load.size()-1,   "load_"+to_string(i));
      domain->addActionFactor(0, action.size()-1,"reboot"+to_string(i));
  }
  domain->setRewardRange(0,num_comps);

  Topology t;
  std::transform(arch.begin(), arch.end(), arch.begin(), ::tolower);
  if(arch == "star") {
      t = Topology::STAR;
  }
  else if(arch == "ring") {
      t = Topology::RING;
  }
  else {
      throw InvalidException("Invalid network topology: " + arch);
  }

  // instantiate sysadmin problem
  Sysadmin sysadmin = boost::make_shared<_Sysadmin>(std::move(domain), t);
  return sysadmin;
}

//
// SimpleSysadmin
//

const double sysadmin::_SimpleSysadmin::REBOOT_PROB = 0.05;
const double sysadmin::_SimpleSysadmin::REBOOT_PENALTY = 0.75;

_SimpleSysadmin::_SimpleSysadmin(Domain domain, Topology network)
: _Sysadmin(domain, network, domain->getNumStateFactors(), domain->getNumActionFactors()) {
  // build factored MDP
  buildFactoredMDP();
}

// Holds for both RING and STAR topologies
void _SimpleSysadmin::buildPlate(Size c, DBNFactor& fas, DBNFactor& fal, LRF& lrf) {
  Domain fasdom = fas->getSubdomain();

  // Define Status (note variable sorting in local scope!)
  Size t = 1;  // index of this computer in subdomain
  Size n = 0;  // index of neighbor in subdomain
  if(c == 0) { // special case: first computer in network
      if(_network == Topology::RING && _num_comps > 1) {
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
          Probability running = 0.45 + 0.5*(2-s.getFactor(n))/2.;
          switch(cur) {
              case (Factor)Status::GOOD:
                  fas->setT(s, a, (Factor)Status::GOOD, running);
                  fas->setT(s, a, (Factor)Status::FAULTY, 1-running);
                  break;
              case (Factor)Status::FAULTY:
                  fas->setT(s, a, (Factor)Status::GOOD, REBOOT_PROB);
                  fas->setT(s, a, (Factor)Status::FAULTY, 1-REBOOT_PROB);
                  break;
          }
      }
  }

  // Define LRF
  State dummy_s;
  Action dummy_a;
  dummy_s.setIndex(static_cast<Size>(Status::FAULTY));
  dummy_a.setIndex(static_cast<Size>(Admin::REBOOT));
  lrf->define(State(), Action(), 1.);  // Status::GOOD, Admin::NOTHING
  lrf->define(State(), dummy_a, 1.-REBOOT_PENALTY);
  lrf->define(dummy_s, dummy_a, 0.-REBOOT_PENALTY);
}

void _SimpleSysadmin::buildFactoredMDP() {
  _fmdp = boost::make_shared<_FactoredMDP>(_domain);

  time_t start_time = time_in_milli();
  for(Size i = 0; i < _num_comps; i++) {
      // create dbn factors for computer `i'
      DBNFactor fas = boost::make_shared<_DBNFactor>(_domain, i);   // status
      LRF lrf = boost::make_shared<_LRF>(_domain); // those have equivalent scopes here
      // Status variable
      fas->addDelayedDependency(i);  // self
      fas->addActionDependency(i);
      if(_network == Topology::RING) { // neighbor
          if(i>0) {
              fas->addDelayedDependency(i-1);
          }
          else {
              fas->addDelayedDependency(_num_comps-1); // close the ring
          }
      }
      else { // STAR
          fas->addDelayedDependency(0); // all depend on the first computer (the `server');
      }
      // LRF
      lrf->addStateFactor(i);       // over status variable
      lrf->addActionFactor(i);
      // allocate tabular storage
      fas->pack();
      lrf->pack();

      // Fill transition and reward function
      buildPlate(i, fas, fas, lrf);
      // add factors for computer `i' to DBN
      _fmdp->addDBNFactor(std::move(fas));
      _fmdp->addLRF(std::move(lrf));
  }

  time_t end_time = time_in_milli();
  LOG_INFO("created factored MDP in " << end_time - start_time << "ms.");
}

State _SimpleSysadmin::begin() {
  _current = State(_domain);
  for(Size i = 0; i < _num_comps; i++) {
      _current.setFactor(i, (Factor) Status::GOOD);
  }
  return _current;
}

Reward _SimpleSysadmin::getReward(const State& s, const Action& a) const {
  Reward r = 0.;
  // over all Status variables
  for(Size i = 0; i < s.size(); i++) {
      // increment for every successful task
      r += static_cast<Reward>(s.getFactor(i) == (Factor) Status::GOOD);
  }
  for(Size j = 0; j < a.size(); j++) {
      r -= static_cast<Reward>(a.getFactor(j) == (Factor) Admin::REBOOT) * REBOOT_PENALTY;
  }
  return r;
}

Sysadmin buildSimpleSysadmin(string arch, Size num_comps) {
  Domain domain = boost::make_shared<_Domain>();
  // variables
  const vector<Status> status {Status::GOOD,   Status::FAULTY};        // 0,1
  const vector<Admin>  action {Admin::NOTHING, Admin::REBOOT};         // 0,1

  for(Size i = 0; i < num_comps; i++) {
      domain->addStateFactor(0, status.size()-1, "status_"+to_string(i));
      domain->addActionFactor(0, action.size()-1,"reboot"+to_string(i));
  }
  domain->setRewardRange(-num_comps*_SimpleSysadmin::REBOOT_PENALTY, num_comps);

  Topology t;
  std::transform(arch.begin(), arch.end(), arch.begin(), ::tolower);
  if(arch == "star") {
      t = Topology::STAR;
  }
  else if(arch == "ring") {
      t = Topology::RING;
  }
  else {
      throw InvalidException("Invalid network topology: " + arch);
  }

  // instantiate sysadmin problem
  Sysadmin sysadmin = boost::make_shared<_SimpleSysadmin>(std::move(domain), t);
  return sysadmin;
}

//
// RddlSysadmin
//

_RddlSysadmin::_RddlSysadmin(Domain domain, const std::multimap<Size,Size>& connectivity, double reboot_prob)
  : _SimpleSysadmin(std::move(domain), Topology::CUSTOM), _connectivity(connectivity), _reboot_prob(reboot_prob) {
  // build factored MDP
  buildFactoredMDP();
}

void _RddlSysadmin::buildPlate(Size c, DBNFactor& fas, DBNFactor& fal, LRF& lrf) {
  using It = decltype(_connectivity)::iterator;
  Domain fasdom = fas->getSubdomain();

  // determine incoming connections
  SizeVec sc;
  pair<It,It> range = _connectivity.equal_range(c);
  std::transform(range.first, range.second, std::back_inserter(sc),
                 [](const pair<Size,Size>& item) { return item.second; });
  std::sort(sc.begin(), sc.end());
  // incoming connections do not include self
  assert(fasdom->getNumStateFactors()-1 == sc.size());

  auto lb = std::lower_bound(sc.begin(), sc.end(), c);
  Size t = lb - sc.begin(); // index of this computer in subdomain

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
          Size n_faulty = 0; // faulty neighbors
          for(int n = 0; n < fasdom->getNumStateFactors(); n++) {
              if(n != t) {
                  n_faulty += s.getFactor(n);
              }
          }
          Size n_running = sc.size() - n_faulty;
          Probability running = 0.45 + 0.5*(1.+n_running)/(1.+sc.size());
          switch(cur) {
              case (Factor)Status::GOOD:
                  fas->setT(s, a, (Factor)Status::GOOD, running);
                  fas->setT(s, a, (Factor)Status::FAULTY, 1-running);
                  break;
              case (Factor)Status::FAULTY:
                  fas->setT(s, a, (Factor)Status::GOOD, _reboot_prob);
                  fas->setT(s, a, (Factor)Status::FAULTY, 1-_reboot_prob);
                  break;
          }
      }
  }

  // Define LRF
  State dummy_s;
  Action dummy_a;
  dummy_s.setIndex(static_cast<Size>(Status::FAULTY));
  dummy_a.setIndex(static_cast<Size>(Admin::REBOOT));
  double payoff = 1.;
  lrf->define(State(), Action(), payoff);  // Status::GOOD, Admin::NOTHING
  lrf->define(State(), dummy_a, payoff-REBOOT_PENALTY);
  lrf->define(dummy_s, dummy_a, 0.-REBOOT_PENALTY);
}

void _RddlSysadmin::buildFactoredMDP() {
  using It = decltype(_connectivity)::iterator;
  _fmdp = boost::make_shared<_FactoredMDP>(_domain);

  time_t start_time = time_in_milli();
  for(Size i = 0; i < _num_comps; i++) {
      // create dbn factors for computer `i'
      DBNFactor fas = boost::make_shared<_DBNFactor>(_domain, i);   // status
      LRF lrf = boost::make_shared<_LRF>(_domain); // those have equivalent scopes here
      // Status variable
      fas->addDelayedDependency(i);  // self
      fas->addActionDependency(i);

      // determine incoming connections
      pair<It,It> range = _connectivity.equal_range(i);
      for(auto it = range.first; it != range.second; ++it) {
          fas->addDelayedDependency(it->second);
      }

      // LRF
      lrf->addStateFactor(i);       // over status variable
      lrf->addActionFactor(i);
      // allocate tabular storage
      fas->pack();
      lrf->pack();

      // Fill transition and reward function
      buildPlate(i, fas, fas, lrf);
      // add factors for computer `i' to DBN
      _fmdp->addDBNFactor(std::move(fas));
      _fmdp->addLRF(std::move(lrf));
  }

  time_t end_time = time_in_milli();
  LOG_INFO("created factored MDP in " << end_time - start_time << "ms.");

}

Sysadmin buildRddlSysadmin(string rddl_file) {
  static const string RDDL_REBOOT = "REBOOT-PROB=";
  static const string RDDL_CONNECT = "CONNECTED(";
  static const Size UPPER_BOUND = 1024; // upper bound on computer number
  Domain domain = boost::make_shared<_Domain>();

  // variables
  const vector<Status> status {Status::GOOD,   Status::FAULTY};        // 0,1
  const vector<Admin>  action {Admin::NOTHING, Admin::REBOOT};         // 0,1

  ifstream file(rddl_file);
  if (!file) {
      throw cpputil::InvalidException("File not found.");
  }

  // basis RDDL parser for connectivity and reboot-prob extraction
  multimap<Size,Size> connectivity;
  vector<bool> comp_list(UPPER_BOUND, false);
  double reboot_prob = _SimpleSysadmin::REBOOT_PROB;
  std::string line;
  while (std::getline(file, line)) {
     line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
     std::size_t found = line.find(RDDL_REBOOT);
     if(found != string::npos) {
         reboot_prob = std::stod(line.substr(found+RDDL_REBOOT.size()));
         LOG_DEBUG("REBOOT_PROB: " << reboot_prob);
         continue;
     }
     // parse connectivity string
     found = line.find(RDDL_CONNECT);
     std::string::size_type offset = 0;
     if(found != string::npos) {
         // silly parsing for now: assumes that computers have two character names x# and are offset from `1'
         Size parent = std::stoul(line.substr(found+RDDL_CONNECT.size()+1), &offset) - 1;
         Size child = std::stoi(line.substr(found+RDDL_CONNECT.size()+offset+3)) - 1;
         assert(parent < UPPER_BOUND && child < UPPER_BOUND);
         connectivity.insert(std::make_pair(child, parent));
         comp_list[parent] = true;
         comp_list[child] = true;
         LOG_DEBUG("Connecting: " << parent << " and " << child);
     }
  }

  // count unique computers
  Size num_comps = std::count(comp_list.begin(), comp_list.end(), true);
  LOG_DEBUG("num_comps: " << num_comps);

  for(Size i = 0; i < num_comps; i++) {
      domain->addStateFactor(0, status.size()-1, "status_"+to_string(i));
      domain->addActionFactor(0, action.size()-1,"reboot"+to_string(i));
  }
  domain->setRewardRange(-num_comps*_SimpleSysadmin::REBOOT_PENALTY, num_comps);

  // instantiate sysadmin problem
  Sysadmin sysadmin = boost::make_shared<_RddlSysadmin>(std::move(domain), connectivity, reboot_prob);
  return sysadmin;
}

} // namespace crl
