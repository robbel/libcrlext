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

#include <sstream>
#include <string.h>
#include <rlglue/Environment_common.h>
#include <rlglue/utils/C/RLStruct_util.h>
#include <rlglue/utils/C/TaskSpec_Parser.h>
#include <crl/crl.hpp>
#include <crl/fdomain.hpp>
#include "crl/glue_env.hpp"
#include "crl/glue_util.hpp"

using namespace std;
using namespace crl;

Environment _env;
Domain _domain_env;
observation_t this_observation;
reward_observation_terminal_t this_reward_observation;
string task_string;
int current_state=0;

const char* env_init()
{    
	_domain_env = getCRLEnvironmentDomain();
	_env = getCRLEnvironment(_domain_env);
	
//	task_specification_t task_spec="2:e:1_[i]_[0,20]:1_[i]_[0,1]:[-1,1]";

	std::ostringstream os;
	os << "2:e:" << _domain_env->getNumStateFactors() << "_[";
	for (Size i=0; i<_domain_env->getNumStateFactors(); i++) {
		os << "i";
		if (i != _domain_env->getNumStateFactors()-1)
			os << ",";
	}
	os << "]";
	for (Size i=0; i<_domain_env->getNumStateFactors(); i++) {
		os << "_[" << _domain_env->getStateRanges()[i].getMin()
		   << "," << _domain_env->getStateRanges()[i].getMax()
		   << "]";
	}
	os << ":" << _domain_env->getNumActionFactors() << "_[";
	for (Size i=0; i<_domain_env->getNumActionFactors(); i++) {
		os << "i";
		if (i != _domain_env->getNumActionFactors()-1)
			os << ",";
	}
	os << "]";
	for (Size i=0; i<_domain_env->getNumActionFactors(); i++) {
		os << "_[" << _domain_env->getActionRanges()[i].getMin()
		   << "," << _domain_env->getActionRanges()[i].getMax()
		   << "]";
	}
	os << ":[" << _domain_env->getRewardRange().getMin()
	   << "," << _domain_env->getRewardRange().getMax() << "]";
	
	
	/* Allocate the observation variable */
	allocateRLStruct(&this_observation, _domain_env->getNumStateFactors(),0,0);
	/* That is equivalent to:
		 this_observation.numInts     =  1;
		 this_observation.intArray    = (int*)calloc(1,sizeof(int));
		 this_observation.numDoubles  = 0;
		 this_observation.doubleArray = 0;
		 this_observation.numChars    = 0;
		 this_observation.charArray   = 0;
	*/
	/* Setup the reward_observation variable */
	this_reward_observation.observation=&this_observation;
	this_reward_observation.reward=0;
	this_reward_observation.terminal=0;

	task_string = os.str();
	cerr << task_string << endl;
	return (char*)(task_string.c_str());
}

const observation_t* env_start()
{
	State s = _env->begin();
	populateState(_domain_env, s, &this_observation);
  	return &this_observation;
}

const reward_observation_terminal_t* env_step(const action_t* this_action)
{
	
	Action a = getAction(_domain_env, this_action);
	Observation o = _env->getObservation(a);
	populateState(_domain_env, o->getState(), &this_observation);
	this_reward_observation.observation = &this_observation;
	this_reward_observation.reward = o->getReward();
	this_reward_observation.terminal = _env->isTerminated();
	return &this_reward_observation;
}

void env_cleanup()
{
	clearRLStruct(&this_observation);
}


