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
  map.reserve(ub);
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
  : _domain(std::move(domain)), _adj_map(std::move(adj_map)) {
  // default initialize this GraphProp
  _num_nodes   = _domain->getNumStateFactors();
  _num_agents  = _domain->getNumActionFactors(),
  _num_targets = 0;
  setParameters(0., 0., 0., 0., 0., 0.);
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

void _GraphProp::buildFactoredMDP() {
  assert(!_agent_locs.empty());
  _fmdp = boost::make_shared<_FactoredMDP>(_domain);

  time_t start_time = time_in_milli();

  // todo..

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
  }
  else {
      LOG_INFO("Non-targeted problem.");
  }
}

BigState _GraphProp::begin() {
  _current = BigState(_domain);
  for(Size i = 0; i < _num_nodes; i++) {
    _current.setFactor(i, static_cast<Factor>(randDouble() < _q0)); // binary state
  }
  return _current;
}

// apply action, obtain resulting state n, update _current
// TODO make this a generic getObservation() for a `FactoredMDPEnvironment' (\see _MDPEnvironment)
BigObservation _GraphProp::getObservation(const Action& a) {
  Reward r = getReward(_current, a);
  // prepare a new state
  BigState new_current = _current;

  // todo .. (\see env_sysadmin.cpp)

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
//      <lambda1 value="25"/>             <!-- Reward model: scaler missing a target node (per target) -->
//      <lambda2 value="10"/>             <!-- Reward model: scaler for reaching a non-target (per non-target) -->
//      <lambda3 value="300"/>            <!-- Reward model: scaler for action cost (per node) -->
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
