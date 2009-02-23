/*
    Copyright 2009 Rutgers University
    Copyright 2009 John Asmuth

    This file is part of CRL:RL-Glue:bayes.

    CRL:RL-Glue:bayes is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CRL:RL-Glue:bayes is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CRL:RL-Glue:bayes.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <cpputil.hpp>
#include <rlglue/Agent_common.h>
#include <crl/crl.hpp>
#include <crl/rmax.hpp>
#include <crl/vi.hpp>
#include <crl/fdomain.hpp>
#include <rlgnmagent.h>
#include <crl/glue_agent.hpp>
#include "crl/outcomes.hpp"
#include "crl/cluster_gibbs.hpp"
#include "crl/dpmem.hpp"
#include "crl/boss.hpp"

using namespace std;
using namespace crl;

int _m;
float _gamma;
Reward _epsilon;

vector<Outcome> the_outcomes;


class _OutcomeClusterAgent : public _Agent {
protected:
	BOSSPlanner _boss_planner;
	OutcomeClusterLearner _cluster_learner;
public:
	_OutcomeClusterAgent(BOSSPlanner boss_planner, OutcomeClusterLearner cluster_learner)
	: _Agent(boss_planner, cluster_learner),
	  _boss_planner(boss_planner), _cluster_learner(cluster_learner) { }
	virtual ~_OutcomeClusterAgent() {
		//_cluster_learner->print();
		//_cluster_learner->inferClusters();
		//_cluster_learner->printClusters();
	}
	virtual Action getAction(const State& s) {
		set<MDP> mdps = _cluster_learner->sampleMDPs(5, 100, 10);
		_boss_planner->setMDPs(mdps);
		_boss_planner->plan();
		Action a = _boss_planner->getAction(s);
		return a;
	}
	virtual void end() {
		Observation o(new _Observation(State(), 0));
		_cluster_learner->observe(_last_state, _last_action, o);
	}
};

void populateOutcomes(Domain domain) {
	vector<int> steps;
	steps.push_back(1);
	Outcome rightOutcome(new _StepOutcome(steps));
	the_outcomes.push_back(rightOutcome);

	State origin(domain);
	origin.setFactor(0, 0);
	Outcome resetOutcome(new _FixedOutcome(origin));
	the_outcomes.push_back(resetOutcome);

	Outcome termOutcome(new _TerminalOutcome());
	the_outcomes.push_back(termOutcome);
}

Agent crl::getCRLAgent(Domain domain) {
	populateOutcomes(domain);
	QTable qtable(new _FQTable(domain));
	BOSSPlanner planner(new _BOSSPlanner(.1, .9, qtable));
	OutcomeClusterLearner mdp_learner(new _OutcomeClusterLearner(domain, the_outcomes, .5));
	/*
	KnownClassifier classifier(new _FKnownClassifier(domain, _m));
	ActionIterator itr(new _ActionIncrementIterator(domain));

	Reward vmax = domain->getRewardRange().getMax();
	if (_gamma < 1)
		vmax = domain->getRewardRange().getMax()/(1-_gamma);

	RMaxMDPLearner rmaxLearner(new _RMaxMDPLearner(mdp_learner, classifier, itr, vmax));
	 */

	//VIPlanner planner(new _FactoredVIPlanner(domain, rmaxLearner, _epsilon, _gamma));
	Agent agent(new _OutcomeClusterAgent(planner, mdp_learner));
	return agent;
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
	try {
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
	}
	catch (cpputil::Exception e) {
		cerr << e << endl << e.trace << endl;
	}
	return 0;
}