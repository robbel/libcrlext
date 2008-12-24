#include <iostream>
#include "cpputil.hpp"
#include "crl/rtdp.hpp"

using namespace std;
using namespace cpputil;
using namespace crl;

Reward _RTDPPlanner::runSimulation(const State& s, Size depth) {
	if (depth >= _max_depth)
		return 0;
		
	//if we've simulated this state too often, terminate the roll-out
	Index count = _s_counts->getValue(s);
	if (count >= _m)
		return 0;
	
	//select an epsilon greedy action
	Action a = _qtable->getBestAction(s);
	if (randDouble() < _explore_epsilon) {
		a = Action(_domain, random()%_domain->getNumActions());
	}
	
	Reward error = 0;
	
	try {
		//sample from the MDP
		State n = _mdp->T(s, a)->sample();
		
		Reward next_error = runSimulation(n, depth+1);
		
		//use the VI planner to perform a backup on s
		ActionIterator aitr = _mdp->A();
		error = _vi_planner->backupState(s, aitr);
		//update the simulation count for this state
		_s_counts->setValue(s, count+1);
		
		if (next_error > error)
			error = next_error;
	}
	catch (DistributionException e) {
		/* reached terminal state.
		 * sampling throws an exception if all next states have probability 0.
		 */
	}
	return error;
}

Action _RTDPPlanner::getAction(const State& s) {
	Size num_runs = 0;
	time_t start_time = time_in_milli();
	time_t time_total = 0;
	
	while (_s_counts->getValue(s) < _m &&
          (_run_limit==0 || num_runs<_run_limit) && //0 for limit means unlimited
          (_time_limit==0 || time_total<_time_limit)) {
		
		Reward error = runSimulation(s);
		
		time_total = time_in_milli()-start_time;
		num_runs++;
		
		if (error < _epsilon) {
//			break;
		}
	}
	
	//always back up the queried state
	ActionIterator aitr = _mdp->A();
	_vi_planner->backupState(s, aitr);
	
	Action a = _qtable->getBestAction(s);
	return a;
}

/**
 * Initialize parameters, as well as created a VI planner using the
 * provided q-table.
 */
_RTDPPlanner::_RTDPPlanner(Domain domain, MDP mdp, QTable qtable,
						   SCountTable s_counts,
				           Reward gamma, Reward epsilon, Index m,
				           Probability explore_epsilon, Size max_depth)
: _domain(domain), _mdp(mdp), _qtable(qtable), _s_counts(s_counts),
  _gamma(gamma), _epsilon(epsilon), _m(m),
  _run_limit(0), _time_limit(0),
  _explore_epsilon(explore_epsilon), _max_depth(max_depth),
  _vi_planner(new _VIPlanner(mdp, epsilon, gamma, _qtable)) {

}

/**
 * This constructor initializes flat q-table and count table, as well
 * as sending up the planner parameters.
 */
_FlatRTDPPlanner::_FlatRTDPPlanner(Domain domain, MDP mdp,
					               Reward gamma, Reward epsilon, Index m,
					               Probability explore_epsilon, Size max_depth)
: _RTDPPlanner(domain, mdp, QTable(new _FQTable(domain, 0)),
			   SCountTable(new _FStateTable<Index>(domain, 0)),
               gamma, epsilon, m, explore_epsilon, max_depth) {

}

