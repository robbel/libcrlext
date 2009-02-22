/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth

    This file is part of CRL:Glue-environments.

    CRL:RL-environments is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CRL:RL-environments is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CRL:RL-environments.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <sstream>
#include <string.h>
#include <rlglue/Environment_common.h>
#include <rlglue/RL_glue.h>
#include <rlgnmenv.h>
#include <cpputil.hpp>
#include <crl/crl.hpp>
#include <crl/fdomain.hpp>
#include "crl/glue_env.hpp"

using namespace std;
using namespace cpputil;
using namespace crl;

int num_links = 5;
int num_types = 2;
Probability type_priors[] = {.5, .5};
Probability probs[] = {.9, .7};
Reward reward_step = -1;
Reward reward_goal = 0;
Reward reward_reset = 0;
bool reset = true; //reset goes to beginning with true, back one step if false

class _ChainEnv : public _Environment {
protected:
	Domain _domain;
	State _current;
	vector<int> _state_types;
public:
	_ChainEnv(const Domain& domain);
	virtual ~_ChainEnv() { }

	virtual State begin();
	virtual bool isTerminated();
	virtual Observation getObservation(const Action& a);
};
typedef boost::shared_ptr<_ChainEnv> ChainEnv;

_ChainEnv::_ChainEnv(const Domain& domain)
: _domain(domain) {
	srand(time(0));
	for (int i=0; i<num_links+1; i++) {
		float r = randDouble();
		float c = 0;
		for (int j=0; j<num_types; j++) {
			c += type_priors[j];
			if (r < c) {
				_state_types.push_back(j);
				break;
			}
		}
		//cerr << _state_types[i];
	}
	//cerr << endl;
}

State _ChainEnv::begin() {
	_current = State(_domain);
	_current.setFactor(0, 0);
	return _current;
}

bool _ChainEnv::isTerminated() {
	return _current.getFactor(0) == num_links;
}

Observation _ChainEnv::getObservation(const Action& a) {
	Reward r = reward_step;
	int link = _current.getFactor(0);
	int action = a.getFactor(0);

	if (link == num_links-1) {
		r = reward_goal;
		_current.setFactor(0, link+1);
		Observation o(new _Observation(_current, r));
		return o;
	}

	Probability p_switch = probs[_state_types[link]];



	if (p_switch > randDouble()) {
		action = 1-action;
	}
	if (action == 0) {
		if (reset || link == 0)
			_current.setFactor(0, 0);
		else
			_current.setFactor(0, link-1);
		r = reward_reset;
	}
	if (action == 1)
		_current.setFactor(0, link+1);
	Observation o(new _Observation(_current, r));
	return o;
}


Domain crl::getCRLEnvironmentDomain() {
	Domain domain(new _Domain());
	domain->addStateFactor(0, num_links);
	domain->addActionFactor(0, 1);
	domain->setRewardRange(reward_step, reward_goal);
	return domain;
}

Environment crl::getCRLEnvironment(Domain domain) {
	ChainEnv env(new _ChainEnv(domain));
	return env;
}

char paramBuf[256];
const char* env_message(const char* inMessage) {
	if (!strcmp(inMessage, "id"))
		return (char*)"poupart-chain";
	else if (!strcmp(inMessage, "param"))
		return paramBuf;
	else if (!strcmp(inMessage, "version"))
		return (char*)"1";
	else if (!strncmp(inMessage, "seed", 4)) {
		long seed = atoi(inMessage+5);
		srand(seed);
	}
	return (char*)"";
 }

int main(int argc, char** argv) {
	if (argc != 1 && argc != 2 && argc != 4 && argc != 5) {
		cerr << "Usage: " << argv[0] << endl;
		return 1;
	}

	paramBuf[0] = '\0';

	glue_main_env(0, 0);

	return 0;
}

