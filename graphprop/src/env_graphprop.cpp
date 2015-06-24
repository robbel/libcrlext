/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <iostream>
#include "crl/env_graphprop.hpp"
#include "logger.hpp"
#include <strxml.hpp>

using namespace std;
using namespace crl;
using namespace crl::graphprop;
using namespace cpputil;

namespace {

/// \brief Parse string into location vector.
/// \param ub Location is tested for inclusion in [0,ub)
void parseLocation(std::string s, Size ub, SizeVec& dest, std::vector<bool>& map) {
  if(s.empty()) {
      //TODO: randomize assignment (and enforce that agentLoc != targetLoc
      throw InvalidException("Location randomization not implemented yet.");
  }
  map.resize(ub);
  std::stringstream stream(s);
  string tok;
  while(std::getline(stream, tok, ',')) {
      Size uloc = std::stoull(tok);
      if(in_pos_interval(uloc, ub)) {
          dest.push_back(uloc);
          map[uloc] = true;
      } else {
          throw InvalidException("Invalid location in location string.");
      }
  }
  std::sort(dest.begin(), dest.end());
}

} // anonymous ns

namespace crl {

_GraphProp::_GraphProp(Domain domain, AdjacencyMap adj_map)
  : _domain(std::move(domain)), _enable_stdout(false), _adj_map(std::move(adj_map)), _beta_t(_domain) {
  // default initialize this GraphProp
  _num_nodes   = _domain->getNumStateFactors();
  _num_agents  = _domain->getNumActionFactors(),
  _num_targets = 0;
  setParameters(0., 0., 0., 0., 0., 0.);

  // incoming connectivity relations
  _AdjacencyMap incoming(_domain);
  incoming.values() = _adj_map->transpose(); // only required for directed graphs
  // build parent scopes
  for(Size i = 0; i < _num_nodes; i++) {
      bool self = true;
      SizeVec sv;
      const auto rowIt = incoming.getRow(i);
      auto iter = rowIt;
      while((iter = std::find(iter, rowIt+_num_nodes, 1)) != rowIt+_num_nodes) {
          auto idx = std::distance(rowIt,iter);
          if(self && idx > i) {
              sv.push_back(i); // include self in parent set
              self = false;
          }
          sv.push_back(idx);
          ++iter;
      }
      if(self) {
        sv.push_back(i);
      }
      _scope_map.insert({i,sv});
  }
}

Reward _GraphProp::getReward(const BigState& s, const Action& a) const {
  Reward cost = 0.;
  Size j = 0;
  // over all state factors
  for(Size i = 0; i < s.size(); i++) {
      if(_agent_active[i]) { // assume non-target node (Vc != Vt)
          cost += _lambda3*a.getFactor(j++) + _lambda2*s.getFactor(i);
      }
      else if(_target_active[i]) { // assume uncontrolled (non-agent) node
          cost += _lambda1*(1-s.getFactor(i));
      }
      else { // a non-target, uncontrolled node
          cost += _lambda2*s.getFactor(i);
      }
  }
  return -cost;
}

// Holds for both controlled and uncontrolled nodes in graph
void _GraphProp::buildPlate(Size i, DBNFactor& fai, LRF& lrf) {
  Domain faidom = fai->getSubdomain();
  const SizeVec& parents = _scope_map[i]; // Note: includes self
  auto loc = std::lower_bound(parents.begin(),parents.end(),i);
  assert(*loc == i);
  const Size locIdx = std::distance(parents.begin(), loc); // local index of `i' in parent scope
  const double d = _del[i]; // recovery rate of ego state
  const auto bIt = _beta_t.getRow(i); // vector of betas (infection rates) from incoming nodes

  _StateIncrementIterator sitr(faidom);
  while(sitr.hasNext()) {
    const State& s = sitr.next();
    const Factor cur = s.getFactor(locIdx);

    // nodes that are uncontrolled have equivalent transitions to controlled ones where action = 0
    assert(faidom->getNumStateFactors() == _scope_map[i].size());
    double prod = 1.;
    Size j = 0;
    for(Size parIdx : _scope_map[i]) {
        prod *= 1. - *(bIt+parIdx) * s.getFactor(j++); // note: no infection without neighbors active
    }

    double pIS = d;        // probability of recovery for this node
    double pSI = 1 - prod; // infection probability

    if(!cur) {
        fai->setT(s, Action(faidom,0), 0, 1-pSI);
        fai->setT(s, Action(faidom,0), 1, pSI);
    }
    else {
        fai->setT(s, Action(faidom,0), 0, pIS);
        fai->setT(s, Action(faidom,0), 1, 1-pIS);
    }

    // handle controlled nodes
    if(_agent_active[i]) {
        // deterministic switch to `1'
        fai->setT(s, Action(faidom,1), 1, 1.0);
    }
  }

  // Define LRF
  State dummy_s; // we don't really need Domain here
  if(_target_active[i]) {
    lrf->define(dummy_s, Action(), -_lambda1);
  }
  else { // either controlled or uncontrolled non-target node
    dummy_s.setIndex(1);
    lrf->define(dummy_s, Action(), -_lambda2);

    if(_agent_active[i]) {
      Action dummy_a;
      dummy_a.setIndex(1);
      lrf->define(dummy_s, dummy_a, -_lambda3-_lambda2);
      lrf->define(State(), dummy_a, -_lambda3);
    }
  }
}

void _GraphProp::buildFactoredMDP() {
  assert(!_agent_locs.empty());
  _fmdp = boost::make_shared<_FactoredMDP>(_domain);

  time_t start_time = time_in_milli();
  Size j = 0;
  // over all state factors
  for(Size i = 0; i < _num_nodes; i++) {
      // create dbn factor for node `i'
      DBNFactor fai = boost::make_shared<_DBNFactor>(_domain,i);
      LRF lrf = boost::make_shared<_LRF>(_domain); // those have equivalent scopes here
      for(Size parent : _scope_map[i]) { // Note: includes self
        fai->addDelayedDependency(parent);
      }
      // LRF
      lrf->addStateFactor(i);
      if(_agent_active[i]) {
        fai->addActionDependency(j);
        lrf->addActionFactor(j++);
      }
      // allocate tabular storage
      fai->pack();
      lrf->pack();

      // Fill transition and reward function
      buildPlate(i, fai, lrf);
      // add factors for node `i' to DBN
      _fmdp->addDBNFactor(std::move(fai));
      _fmdp->addLRF(std::move(lrf));
  }

  time_t end_time = time_in_milli();
  LOG_INFO("created factored MDP in " << end_time - start_time << "ms.");
}

void _GraphProp::setAgentLocs(Size num_agents, std::string locs) {
  assert(_num_agents == num_agents);
  _agent_locs.clear();
  _agent_active.clear();

  parseLocation(locs, _num_nodes, _agent_locs, _agent_active);
  if(cpputil::has_intersection(_agent_locs.begin(), _agent_locs.end(), _target_locs.begin(), _target_locs.end())) {
    throw InvalidException("Agent set and Target set have to be distinct.");
  }
  if(_agent_locs.size() != num_agents) {
    throw InvalidException("Agent number is different from those in location string.");
  }
}

void _GraphProp::setTargetLocs(Size num_targets, std::string locs) {
  _num_targets = num_targets;
  _target_locs.clear();
  _target_active.clear();

  if(_num_targets > 0) {
      parseLocation(locs, _num_nodes, _target_locs, _target_active);
      if(cpputil::has_intersection(_agent_locs.begin(), _agent_locs.end(), _target_locs.begin(), _target_locs.end())) {
          throw InvalidException("Agent set and Target set have to be distinct.");
      }
      if(_target_locs.size() != num_targets) {
        throw InvalidException("Target number is different from those in location string.");
      }
  }
  else {
      _target_active.resize(_num_nodes);
      LOG_INFO("Non-targeted problem.");
  }
}

void _GraphProp::setParameters(double beta0, double del0, double lambda1, double lambda2, double lambda3, double q0) {
  _beta0 = beta0;
  _del0 = del0;
  _lambda1 = lambda1;
  _lambda2 = lambda2;
  _lambda3 = lambda3;
  _q0 = q0;

  // simple, regular diffusion model (consistent across graph) based on _beta0
  // TODO: add hetero diffusion
  const auto& adj_vec = _adj_map->values();
  auto& beta_vec = _beta_t.values();
  auto iter = adj_vec.begin();
  while((iter = std::find(iter, adj_vec.end(), 1)) != adj_vec.end()) {
      auto idx = std::distance(adj_vec.begin(),iter);
      beta_vec[idx] = _beta0;
      ++iter;
  }
  // transpose beta vector
  _beta_t.values() = _beta_t.transpose();
  // simple recovery model based on _del0
  _del = std::vector<double>(_num_nodes, _del0);
}

BigState _GraphProp::begin() {
  _current = BigState(_domain);
  for(Size i = 0; i < _num_nodes; i++) {
    _current.setFactor(i, static_cast<Factor>(randDouble() < _q0)); // binary state
  }
  if(_enable_stdout) {
      std::cout << "S: " << _current << std::endl;
  }
  return _current;
}

// apply action, obtain resulting state n, update _current
// TODO make this a generic getObservation() for a `FactoredMDPEnvironment' (\see _MDPEnvironment)
BigObservation _GraphProp::getObservation(const Action& ja) {
  Reward r = getReward(_current, ja);
  // prepare a new state
  BigState new_current = _current;

  const _DBN& T = _fmdp->T();
  for(int f = 0; f < T.size(); f++) {
      const DBNFactor& fa = T.factor(f);
      const ProbabilityVec& pvec = fa->T(_current, ja);
#if !NDEBUG
//      Probability sum = std::accumulate(pvec.begin(), pvec.end(), 0.);
//      assert(approxEq(sum,1.));
//      std::cout << "i:" << f << " [ ";
//      const SizeVec& parents = _scope_map[f];
//      for(auto pa : parents) {
//          std::cout << pa << " ";
//      }
//      std::cout << "]=" << fa->mapState(_current, State()) << " " << fa->mapAction(ja) << ": " << std::endl;
//      for(auto pr : pvec) {
//          std::cout << pr << " ";
//      }
//      std::cout << std::endl;
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

  if(_enable_stdout) {
      std::cout << "S: " << new_current << std::endl;
  }

  _current = std::move(new_current);
  BigObservation o = boost::make_shared<_BigObservation>(_current, r);
  return o;
}

// The GraphProp layout:
//    <GraphProp>
//      <Nodes   count="100"/>            <!-- Number of nodes in graph (must match supplied graph) -->
//      <Agents  count="3">0,1,2</Agents> <!-- Note: vertex selection string is optional -->
//      <Targets count="0"></Targets>     <!-- Note: vertex selection string is optional -->
//      <Actions count="2"/>              <!-- Number of actions for each controlled node (currently assumed binary) -->
//      <beta0   value="0.2"/>            <!-- Default infection transmission probability -->
//      <del0    value="0.2"/>            <!-- Default recovery probability -->
//      <lambda1 value="300"/>            <!-- Reward model: scaler missing a target node (per missed target) -->
//      <lambda2 value="10"/>             <!-- Reward model: scaler for reaching a non-target (per non-target) -->
//      <lambda3 value="25"/>             <!-- Reward model: scaler for action cost (per agent with action=1) -->
//      <q0      value="0.5"/>            <!-- Default infection probability -->
//    </GraphProp>

GraphProp readGraphProp(std::istream& cfg, std::istream& graph) {
  if(!cfg || !graph) {
      return nullptr;
  }

  // read problem layout from cfg xml file
  XMLObject xobj(cfg);
  if(xobj.getName() != "GraphProp")
      throw xml_exception("Expected <GraphProp>");
  // problem formulation
  XMLObject nodes = xobj["Nodes"];
  int num_nodes = std::stoi(nodes("count"));
  XMLObject agents = xobj["Agents"];
  int num_agents = std::stoi(agents("count"));
  string agentAssignments = agents.size() != 0 ? agents.getText() : ""; // if an assignment string is given
  XMLObject targets = xobj["Targets"];
  int num_targets = std::stoi(targets("count"));
  string targetAssignments = targets.size() != 0 ? targets.getText() : ""; // if an assignment string is given
  XMLObject actions = xobj["Actions"];
  int num_actions = std::stoi(actions("count"));
  if(num_actions != 2) {
      throw xml_exception("Only binary actions supported currently");
  }
  // parameters
  XMLObject beta = xobj["beta0"];
  double beta0 = std::stod(beta("value"));
  XMLObject del = xobj["del0"];
  double del0 = std::stod(del("value"));
  XMLObject l1 = xobj["lambda1"];
  double lambda1 = std::stod(l1("value"));
  XMLObject l2 = xobj["lambda2"];
  double lambda2 = std::stod(l2("value"));
  XMLObject l3 = xobj["lambda3"];
  double lambda3 = std::stod(l3("value"));
  XMLObject q = xobj["q0"];
  double q0 = std::stod(q("value"));

  // construct domain
  Domain domain = boost::make_shared<_Domain>();
  for(int i = 0; i < num_nodes; i++) {
      domain->addStateFactor(0, 1, "n_"+to_string(i)); // binary states
  }
  // two types of actions: '0' for left, '1' for right
  for(int i = 0; i < num_agents; i++) {
      domain->addActionFactor(0, 1, "agent_"+to_string(i)); // binary actions for now
  }
  // compute reward range
  int num_reg = num_nodes - num_targets;
  assert(num_reg >= 0 && num_reg <= num_nodes);
  double max_cost = num_targets * (lambda3 + lambda1) + num_reg * (lambda3 + lambda2);
  domain->setRewardRange(-max_cost, 0.);

  // read graph description
  AdjacencyMap adj_map = boost::make_shared<_AdjacencyMap>(domain);
  std::vector<int>& vals = adj_map->values();
  // read the first line
  std::string header;
  std::getline(graph, header);
  int h_nodes = std::stoi(header);
  if(h_nodes != num_nodes) {
      throw cpputil::SizeException(num_nodes, h_nodes, "Graph file and cfg file do not match.");
  }
  // read the graph adjacency description
  std::copy(std::istream_iterator<int>(graph),
            std::istream_iterator<int>(),
            vals.begin());
  assert(vals.size() == num_nodes*num_nodes);

  // instantiate grp problem
  GraphProp grp = boost::make_shared<_GraphProp>(std::move(domain), std::move(adj_map));
  grp->setAgentLocs(num_agents, agentAssignments);
  grp->setTargetLocs(num_targets, targetAssignments);
  grp->setParameters(beta0, del0, lambda1, lambda2, lambda3, q0);
  grp->buildFactoredMDP();

  return grp;
}

} // namespace crl
