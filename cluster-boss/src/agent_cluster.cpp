/*
    Copyright 2009 Rutgers University
    Copyright 2009 John Asmuth
    Copyright 2015 Philipp Robbel

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
#include <diastream.hpp>
#include <gsl/gsl_rng.h>
#include <cpputil.hpp>
#include <rlglue/Agent_common.h>
#include <crl/crl.hpp>
#include <crl/rmax.hpp>
#include <crl/vi.hpp>
#include <crl/flat_tables.hpp>
#include <rlgnmagent.h>
#include <crl/glue_agent.hpp>
#include "crl/outcomes.hpp"
#include "crl/cluster_gibbs.hpp"
#include "crl/dpmem.hpp"
#include "crl/boss.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

gsl_rng* gsl_random = 0;

double _gamma;
Reward _epsilon;

int _m = 10;
float _alpha = .5;
Size _t_priors = 1;

float _alpha_prior = 1;
float _beta_prior = 0;

int _burn_period = 500;
int _sample_spacing = 50;
int _num_samples = 5;

//0 for chain
//1 for marble maze
//2 for littman chain
int _domain_type = 0;

bool _do_rmax; //instead of BOSS

vector<Outcome> the_outcomes;

class _OutcomeClusterAgent : public _Agent {
protected:
	BOSSPlanner _boss_planner;
	OutcomeClusterLearner _cluster_learner;
	Size num_steps;
public:
	_OutcomeClusterAgent(BOSSPlanner boss_planner, OutcomeClusterLearner cluster_learner)
	: _Agent(boss_planner, cluster_learner),
	  _boss_planner(boss_planner), _cluster_learner(cluster_learner) {
		_cluster_learner->setGSLRandom(gsl_random);
		num_steps = 0;
	}
	void drawMDPs(Size count) {
		vector<MDP> mdps = _boss_planner->getMDPs();
		ContainerIterator<MDP,vector<MDP> > itr(mdps);
		ostringstream tos;
		tos << "dia/mdp-" << _domain_type << "," << _do_rmax << ":" << count << ".dia";
		diastream os(tos.str().c_str());
		os << DiaBeginDoc();
		int s_counter = 0;
		while (itr.hasNext()) {
			MDP mdp = itr.next();
			//mdp->printXML(cerr);
			ClusterMDP cmdp = getClusterMDP(mdp);
			_cluster_learner->makeClusterBOSSVis(os, s_counter++, cmdp);
		}
//		cerr << endl;
		os << DiaEndDoc();
		os.close();
	}
	virtual ~_OutcomeClusterAgent() {
		//_cluster_learner->print();
		//_cluster_learner->inferClusters();
		//_cluster_learner->printClusters();


	}
	virtual bool observe(const Observation& o) {
		bool learned;
		num_steps++;
		if ((learned = _Agent::observe(o))) {
			vector<MDP> mdps = _cluster_learner->sampleMDPs(_num_samples, _burn_period, _sample_spacing, _do_rmax);

			for (Size i=0; i<mdps.size(); i++) {
				if (_domain_type != 0)
					getClusterMDP(mdps[i])->setUseBeta(false);
				if (_do_rmax) {
					getClusterMDP(mdps[i])->setRmaxM(_m);
				}
			}
//			vector<MDP> annealed_mdps = _cluster_learner->sampleMDPs(_num_samples, _burn_period, _sample_spacing, true);
			_boss_planner->setMDPs(mdps);
			//if (_domain_type == 1)
			drawMDPs(num_steps);
			_boss_planner->plan();
		}
		return learned;
	}
	virtual Action getAction(const State& s) {
		_last_action = _boss_planner->getAction(s);
//		cerr << s << " <- " << _last_action << endl;
		return _last_action;
	}
	virtual void end() {
//		Observation o(new _Observation(State(), 0));
//		_cluster_learner->observe(_last_state, _last_action, o);
//		_cluster_learner->printClusters();
	}
};

void populateOutcomes(Domain domain) {
	if (_domain_type == 0) {
		vector<int> steps1;
		steps1.push_back(1);
		Outcome rightOutcome(new _StepOutcome(domain, steps1, true));
		the_outcomes.push_back(rightOutcome);

		State origin(domain);
		origin.setFactor(0, 0);
		Outcome resetOutcome(new _FixedOutcome(origin));
		the_outcomes.push_back(resetOutcome);
	}
	if (_domain_type == 1) {
		vector<int> steps;
		steps.push_back(0);
		steps.push_back(0);
		steps.push_back(0);

		steps[0] = 0;
		steps[1] = 0;
		Outcome stillOutcome(new _StepOutcome(domain, steps, false));
		the_outcomes.push_back(stillOutcome);

		steps[0] = 0;
		steps[1] = -1;
		Outcome northOutcome(new _StepOutcome(domain, steps, true));
		the_outcomes.push_back(northOutcome);

		steps[0] = 1;
		steps[1] = 0;
		Outcome eastOutcome(new _StepOutcome(domain, steps, true));
		the_outcomes.push_back(eastOutcome);

		steps[0] = 0;
		steps[1] = 1;
		Outcome southOutcome(new _StepOutcome(domain, steps, true));
		the_outcomes.push_back(southOutcome);

		steps[0] = -1;
		steps[1] = 0;
		Outcome westOutcome(new _StepOutcome(domain, steps, true));
		the_outcomes.push_back(westOutcome);
	}
	if (_domain_type == 2) {
		vector<int> steps;
		steps.push_back(0);

		steps[0] = -1;
		Outcome leftOutcome(new _StepOutcome(domain, steps, true));
		the_outcomes.push_back(leftOutcome);

		steps[0] = 1;
		Outcome rightOutcome(new _StepOutcome(domain, steps, true));
		the_outcomes.push_back(rightOutcome);

		steps[0] = 0;
		Outcome stillOutcome(new _StepOutcome(domain, steps, false));
		the_outcomes.push_back(stillOutcome);

//		Outcome purgatoryOutcome(new _TerminalOutcome());
//		the_outcomes.push_back(purgatoryOutcome);
	}
}

Agent crl::getCRLAgent(Domain domain) {
	populateOutcomes(domain);
	QTable qtable(new _FQTable(domain));

	BOSSPlanner planner(new _BOSSPlanner(_epsilon, _gamma, qtable));
	OutcomeClusterLearner mdp_learner(new _OutcomeClusterLearner(domain, the_outcomes, _alpha, _m, _t_priors, _alpha_prior, _beta_prior));

	Agent agent(new _OutcomeClusterAgent(planner, mdp_learner));
	return agent;
}

StateMapper crl::getStateMapper() {
	return nullptr;
}

char params[256];
const char* agent_message(const char* inMessage) {
	if (strcmp(inMessage,"id")==0)
		return (char*)"ClusterBOSS";
	if (strcmp(inMessage,"version")==0)
		return (char*)"1";
	if (strcmp(inMessage,"param")==0)
		return params;
	if (!strncmp(inMessage, "seed", 4)) {
		long seed = atoi(inMessage+5);
		srand(seed);
		gsl_rng_set(gsl_random, seed+1);
	}
	return (char*)"";
}


int main(int argc, char** argv) {
	try {
		srand(time(0));

		if (argc != 13 && argc != 14) {
			cerr << "Usage: " << argv[0] << "<alpha> <gamma> <epsilon> <m> <t_priors> <alpha> <beta> <burn> <spacing> <samples> <type> [host:port]" << endl;
			return 1;
		}
		_alpha = atof(argv[1]);
		_gamma = atof(argv[2]);
		_epsilon = atof(argv[3]);
		_m = atoi(argv[4]);
		_t_priors = atoi(argv[5]);
		_alpha_prior = atof(argv[6]);
		_beta_prior = atof(argv[7]);
		_burn_period = atoi(argv[8]);
		_sample_spacing = atoi(argv[9]);
		_num_samples = atoi(argv[10]);
		_domain_type = atoi(argv[11]);
		_do_rmax = atoi(argv[12]);

		char* host = 0;
		short port = 0;
		if (argc == 14) {
			host = strtok(argv[13], ":");
			port = atoi(strtok(0, ":"));
		}

		if (!_do_rmax) {
			sprintf(params, "gamma=%f epsilon=%f burn=%d spacing=%d samples=%d type=%d", _gamma, _epsilon, _burn_period, _sample_spacing, _num_samples, _domain_type);
		}
		else {
			sprintf(params, "rmax gamma=%f epsilon=%f burn=%d type=%d", _gamma, _epsilon, _burn_period, _domain_type);
			_num_samples = 1;
		}

	  	gsl_random = gsl_rng_alloc(gsl_rng_taus2);
		glue_main_agent(host, port);
		gsl_rng_free(gsl_random);
	}
	catch (const cpputil::Exception& e) {
		cerr << e << endl << e.trace << endl;
	}
	return 0;
}
