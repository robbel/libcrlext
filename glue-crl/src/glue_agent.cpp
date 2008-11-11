/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth

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
#include <crl/fdomain.hpp>
#include "crl/glue_agent.hpp"
#include "crl/glue_util.hpp"

using namespace crl;
using namespace std;

Agent _agent;
Domain _domain_agent;
action_t _this_action;
action_t _last_action;
observation_t _last_observation;
task_spec_struct _tss;

void agent_init(const char* task_spec)
{
	/*Seed the random number generator*/
	srand(time(0));
	/*Here is where you might allocate storage for parameters (value function or policy, last action, last observation, etc)*/
	
	/*Here you would parse the task spec if you felt like it*/
	parse_task_spec(task_spec, &_tss);
	
	_domain_agent = Domain(new _Domain());
	for (int i=0; i<_tss.num_discrete_obs_dims; i++) {
		int min = (int)_tss.obs_mins[i];
		int max = (int)_tss.obs_maxs[i];
		_domain_agent->addStateFactor(min, max);
	}
	for (int i=0; i<_tss.num_discrete_action_dims; i++) {
		int min = (int)_tss.action_mins[i];
		int max = (int)_tss.action_maxs[i];
		_domain_agent->addActionFactor(min, max);
	}
	_domain_agent->setRewardRange(_tss.reward_min, _tss.reward_max);
	
	_agent = getCRLAgent(_domain_agent);
		
	/*Allocate memory for a one-dimensional integer action using utility functions from RLStruct_util*/
	allocateRLStruct(&_this_action, _domain_agent->getNumActionFactors(), 0, 0);

}

const action_t* agent_start(const observation_t* this_observation) {
	State s = getState(_domain_agent, this_observation);
	_agent->begin(s);
	Action a;
	if (_agent)
		a = _agent->getAction(s);
	else
		a = Action(_domain_agent, 0);
	populateAction(_domain_agent, a, &_this_action);
	
	/* In a real action you might want to store the last observation and last action*/
	replaceRLStruct(&_this_action, &_last_action);
	replaceRLStruct(this_observation, &_last_observation);
	
	return &_this_action;
}

const action_t* agent_step(double reward, const observation_t* this_observation) {
	/* This agent  returns 0 or 1 randomly for its action */
	/* This agent always returns a random number, either 0 or 1 for its action */


	State n = getState(_domain_agent, this_observation);
	Observation o(new _Observation(n, reward));
	if (_agent);
	_agent->observe(o);
	
	Action a;
	if (_agent)
		a = _agent->getAction(n);
	else
		a = Action(_domain_agent, 0);
	populateAction(_domain_agent, a, &_this_action);
        
	/* In a real action you might want to store the last observation and last action*/
	replaceRLStruct(&_this_action, &_last_action);
	replaceRLStruct(this_observation, &_last_observation);
	
	return &_last_action;
}

void agent_end(double reward) {
	State s;
	Observation o(new _Observation(s, reward));
	if (_agent) {
		_agent->observe(o);
		_agent->end();
	}
	clearRLStruct(&_last_action);
	clearRLStruct(&_last_observation);
}

void agent_cleanup() {
	clearRLStruct(&_this_action);
	clearRLStruct(&_last_action);
	clearRLStruct(&_last_observation);
}

