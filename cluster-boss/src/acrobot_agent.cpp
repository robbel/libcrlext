/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth
    Copyright 2015 Philipp Robbel

    This file is part of CRL:RL-Glue:agents.

    CRL:RL-Glue:agents is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CRL:RL-Glue:agents is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CRL:RL-Glue:agents.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <rlglue/Agent_common.h>
#include <crl/crl.hpp>
#include <crl/rmax.hpp>
#include <crl/rtdp.hpp>
#include <crl/flat_tables.hpp>
#include <rlgnmagent.h>
#include "crl/glue_agent.hpp"

using namespace std;
using namespace crl;

int _m;
float _gamma;
Reward _epsilon;

Agent crl::getCRLAgent(Domain domain) {
	HMDPLearner mdp_learner(new _HMDPLearner(domain));
	KnownClassifier classifier(new _HKnownClassifier(domain, _m));
	ActionIterator itr(new _ActionIncrementIterator(domain));

	Reward vmax = domain->getRewardRange().getMax();
	if (_gamma < 1)
		vmax = domain->getRewardRange().getMax()/(1-_gamma);

	RMaxMDPLearner rmaxLearner(new _RMaxMDPLearner(mdp_learner, classifier, itr, vmax));
/*
	float gamma = .9;
	Reward epsilon = .01;
	int m = 100;
	int run_limit = 0;
	int time_limit = 0;
	int h = 1;
*/
	/*
	RTDPPlanner rtdp_planner(new _FlatRTDPPlanner(domain, rmaxLearner, gamma, epsilon, m, h, 50));
	rtdp_planner->setRunLimit(run_limit);
	rtdp_planner->setTimeLimit(time_limit);
	Planner planner = rtdp_planner;
	*/

	VIPlanner planner(new _FlatVIPlanner(domain, rmaxLearner, _epsilon, _gamma));
	Agent agent(new _Agent(planner, rmaxLearner));
	return agent;
}

class _AcrobotMapper : public _StateMapper {
	taskspec_t* _task_spec;
public:
	virtual ~_AcrobotMapper() { }
	virtual Domain getDomain(Domain old_domain, taskspec_t* task_spec) {
		_task_spec = task_spec;
		Domain domain(new _Domain());
		domain->addStateFactor(-20, 20);
		domain->addStateFactor(-20, 20);
		domain->addStateFactor(-10, 10);
		domain->addStateFactor(-10, 10);
		domain->addActionFactor(0, 7);
		domain->setRewardRange(old_domain->getRewardRange().getMin(),old_domain->getRewardRange().getMax());
		return domain;
	}
	virtual State getState(Domain domain, const observation_t* obs) {

		State s(domain);

		for (Size i=0; i<domain->getNumStateFactors(); i++) {
			cerr << obs->doubleArray[i] << " ";
			Index to_range = domain->getStateRanges()[i].getSpan();
			Index to_offset = domain->getStateRanges()[i].getMin();
			double from_range = _task_spec->double_observations[i].max-_task_spec->double_observations[i].min;
			double from_offset = _task_spec->double_observations[i].min;
			double r = (obs->doubleArray[i]-from_offset)/from_range;
			Index f = r*to_range+to_offset;
			s.setFactor(i, f);
		}
		cerr << endl;
		cerr << s << endl;
		return s;
	}
};

StateMapper crl::getStateMapper() {
	return StateMapper(new _AcrobotMapper());
}

char params[256];
const char* agent_message(const char* inMessage) {
	if (strcmp(inMessage,"id")==0)
		return (char*)"vi_rmax";
	if (strcmp(inMessage,"version")==0)
		return (char*)"1";
	if (strcmp(inMessage,"param")==0)
		return params;
	if (!strncmp(inMessage, "seed", 4)) {
		long seed = atoi(inMessage+5);
		srand(seed);
	}
	return (char*)"";
}


int main(int argc, char** argv) {
	srand(time(0));

	if (argc != 4 && argc != 5) {
		cerr << "Usage: " << argv[0] << " <m> <gamma> <epsilon> [host:port]" << endl;
		return 1;
	}
	_m = atoi(argv[1]);
	_gamma = atof(argv[2]);
	_epsilon = atof(argv[3]);
	char* host = 0;
	short port = 0;
	if (argc == 5) {
		host = strtok(argv[4], ":");
		port = atoi(strtok(0, ":"));
	}

	sprintf(params, "m=%d gamma=%f epsilon=%f", _m, _gamma, _epsilon);

	glue_main_agent(host, port);

	return 0;
}
