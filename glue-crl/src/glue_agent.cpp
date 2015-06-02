/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth
    Copyright 2015 Philipp Robbel

    This file is part of CRL:RL-Glue.

    CRL:RL-Glue is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CRL:RL-Glue is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CRL:RL-Glue.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <string.h>
#include <rlglue/Agent_common.h>
#include <rlglue/utils/C/RLStruct_util.h>
#include <rlglue/utils/C/TaskSpec_Parser.h>
#include <crl/crl.hpp>
#include "crl/glue_agent.hpp"
#include "crl/glue_util.hpp"

using namespace crl;
using namespace std;

/// \brief (Global) \a crl::Agent used for interaction with rl-glue
Agent _agent;
/// \brief (Global) \a crl::Domain used for interaction with rl-glue
Domain _domain_agent;
StateMapper _state_mapper;
BigStateMapper _big_state_mapper;
action_t _this_action;
action_t _last_action;
observation_t _last_observation;
taskspec_t _tss;

//
// The interface from libcrl to rl-glue
//

///
/// \brief Initialize a \a crl::Agent given the task spec
/// As per rl-glue spec
/// \note The current agent must implement getCRLAgent() and (optionally) getStateMapper()
/// \see getCRLAgent(), getStateMapper()
///
extern "C" void agent_init(const char* task_spec) {
  /*Seed the random number generator*/
  srand(time(0));
  /*Here is where you might allocate storage for parameters (value function or policy, last action, last observation, etc)*/

  /*Here you would parse the task spec if you felt like it*/
  decode_taskspec(&_tss, task_spec);

  if (_tss.num_int_observations == 0) {
      _tss.num_int_observations = 4;
      _tss.int_observations = new int_range_t[4];
      for (int i=0; i<2; i++) {
          _tss.int_observations[i].min = -10;
          _tss.int_observations[i].max = 10;
      }
      for (int i=2; i<4; i++) {
          _tss.int_observations[i].min = -3;
          _tss.int_observations[i].max = 3;
      }
  }

  _domain_agent = Domain(new _Domain());
  for (int i=0; i<_tss.num_int_observations; i++) {
      int min = (int)_tss.int_observations[i].min;
      int max = (int)_tss.int_observations[i].max;
      _domain_agent->addStateFactor(min, max);
  }
  for (int i=0; i<_tss.num_int_actions; i++) {
      int min = (int)_tss.int_actions[i].min;
      int max = (int)_tss.int_actions[i].max;
      _domain_agent->addActionFactor(min, max);
  }
  _domain_agent->setRewardRange(_tss.reward.min, _tss.reward.max);

  /*Allocate memory for a one-dimensional integer action using utility functions from RLStruct_util*/
  allocateRLStruct(&_this_action, _domain_agent->getNumActionFactors(), 0, 0);

  _state_mapper = getStateMapper();
  if (!_state_mapper) {
      if(_domain_agent->isBig()) {
          _big_state_mapper = boost::make_shared<_BigStateMapper>();
      }
      else {
          _state_mapper = boost::make_shared<_StateMapper>();
      }
  }

  if(_big_state_mapper) {
      _domain_agent = _big_state_mapper->getDomain(_domain_agent, &_tss);
  }
  else {
      _domain_agent = _state_mapper->getDomain(_domain_agent, &_tss);
  }

  _agent = getCRLAgent(_domain_agent);
}

// helper function
template<class S>
Action agent_start(const observation_t* this_observation, const StateMapperBase<S>& mapper) {
  S s = mapper->getState(_domain_agent, this_observation);
  _agent->begin(s);
  Action a;
  if (_agent)
          a = _agent->getAction(s);
  else
          a = Action(_domain_agent, 0);
  return a;
}
///
/// \brief Initialize the \a crl::Agent given the initial observation from the environment.
/// As per rl-glue spec
///
extern "C" const action_t* agent_start(const observation_t* this_observation) {
  Action a;
  if(_big_state_mapper) {
      a = agent_start(this_observation, _big_state_mapper);
  }
  else {
      a = agent_start(this_observation, _state_mapper);
  }
  populateAction(_domain_agent, a, &_this_action);

  /* In a real action you might want to store the last observation and last action*/
  replaceRLStruct(&_this_action, &_last_action);
  replaceRLStruct(this_observation, &_last_observation);

  return &_this_action;
}

// helper function
template<class S, class O>
Action agent_step(double reward, const observation_t* this_observation, const StateMapperBase<S>& mapper) {
  S n = mapper->getState(_domain_agent, this_observation);
  Observation o(new O(n, reward));
  if (_agent)
    _agent->observe(o);
  Action a;
  if (_agent)
    a = _agent->getAction(n);
  else
    a = Action(_domain_agent, 0);
  return a;
}
///
/// \brief Step the \a crl::Agent and return an action.
/// As per rl-glue spec
/// \note If no agent has been created, this will return the 0-action.
///
extern "C" const action_t* agent_step(double reward, const observation_t* this_observation) {
  Action a;
  if(_big_state_mapper) {
      a = agent_step<BigState,_BigObservation>(reward, this_observation, _big_state_mapper);
  }
  else {
      a = agent_step<State,_Observation>(reward, this_observation, _state_mapper);
  }
  populateAction(_domain_agent, a, &_this_action);

  /* In a real action you might want to store the last observation and last action*/
  replaceRLStruct(&_this_action, &_last_action);
  replaceRLStruct(this_observation, &_last_observation);

  return &_last_action;
}

///
/// \brief Pass the last reward (with the empty state!) to the agent.
/// As per rl-glue spec
///
extern "C" void agent_end(double reward) {
  State s;
  Observation o(new _Observation(s, reward));
  if (_agent) {
      _agent->observe(o); // FIXME should this be observed? After all, this is the null state!
      _agent->end();
  }
  clearRLStruct(&_last_action);
  clearRLStruct(&_last_observation);
}

extern "C" void agent_cleanup() {
  clearRLStruct(&_this_action);
  clearRLStruct(&_last_action);
  clearRLStruct(&_last_observation);
}

