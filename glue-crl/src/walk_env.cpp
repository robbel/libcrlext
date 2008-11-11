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

bool oo = false;
int range = 10;
float reward = 1;
float slip = .2;

class _WalkEnv : public _Environment {
protected:
	Domain _domain;
	State _current;
public:
	_WalkEnv(const Domain& domain);
	virtual ~_WalkEnv() { }
	
	virtual State begin();
	virtual bool isTerminated();
	virtual Observation getObservation(const Action& a);
};
typedef boost::shared_ptr<_WalkEnv> WalkEnv;

_WalkEnv::_WalkEnv(const Domain& domain)
: _domain(domain) {
	
}

State _WalkEnv::begin() {
	_current = State(_domain);
	_current.setFactor(0, 0);
	return _current;
}

bool _WalkEnv::isTerminated() {
	return _current.getFactor(0) == -1*range || _current.getFactor(0) == range;
}

Observation _WalkEnv::getObservation(const Action& a) {
	Reward r = reward;
	int delta = 1 ;
	if (randDouble()<slip)
		delta = -1;
	
	if (a.getFactor(0) == 0) {
		r *= -1;
		delta *= -1;
	}
	
	int l = _current.getFactor(0)+delta;
	_current = State(_domain);
	_current.setFactor(0, l);
	Observation o(new _Observation(_current, r));
	
	if (oo) {
		ostringstream os;
		os << "<State>"
		   <<  "<Object>" 
		   <<   "<Attribute type=\"x\" value=\"" << l << "\"/>"
		   <<  "</Object>"
		   << "</State>";
		   
	}		
	return o;
}


Domain crl::getCRLEnvironmentDomain() {
	Domain domain(new _Domain());
	domain->addStateFactor(-1*range, range);
	domain->addActionFactor(0, 1);
	domain->setRewardRange(-1*reward, reward);
	return domain;
}

Environment crl::getCRLEnvironment(Domain domain) {
	WalkEnv env(new _WalkEnv(domain));
	return env;
}

char paramBuf[256];
const char* env_message(const char* inMessage) {
	if (!strcmp(inMessage, "id"))
		return (char*)"walk";
	else if (!strcmp(inMessage, "param"))
		return paramBuf;
	else if (!strcmp(inMessage, "version"))
		return (char*)"1";
	return (char*)"";
}

int main(int argc, char** argv) {
	if (argc != 1 && argc != 2 && argc != 4 && argc != 5) {
		cerr << "Usage: " << argv[0] << " [range reward slip] [-oo]" << endl;
		return 1;	
	}
	if (argc >= 4) {
		range = atoi(argv[1]);
		reward = atof(argv[2]);	
		slip = atof(argv[3]);
		if (argc == 5) {
			oo = !strcmp(argv[4], "-oo");
		}
	}
	else if (argc == 2) {
		oo = !strcmp(argv[1], "-oo");
	}
	
	sprintf(paramBuf, "range=%d reward=%f slip=%f", range, reward, slip);
	
	glue_main_env(0, 0);
	
	return 0;
}

