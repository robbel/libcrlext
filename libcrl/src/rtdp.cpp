#include <iostream>
#include "cpputil.hpp"
#include "crl/rtdp.hpp"

using namespace std;
using namespace cpputil;
using namespace crl;

void _RTDPPlanner::runSimulation(const State& s, StateSet ss, Size depth) {
	if (depth >= _max_depth)
		return;
		
//	ss->insert(s);
		
	Action a = _qtable->getBestAction(s);
	if (randDouble() < _explore_epsilon) {
		a = Action(_domain, random()%_domain->getNumActions());
	}
	
	try {
		State n = _mdp->T(s, a)->sample();
		
		runSimulation(n, ss, depth+1);
		
		ActionIterator aitr = _mdp->A();
		_vi_planner->backupState(s, aitr);
	}
	catch (DistributionException e) {
//		cerr << "reached terminal state" << endl;
	}
}

Action _RTDPPlanner::getAction(const State& s) {
	Size num_runs = 0;
	time_t start_time = time_in_milli();
	time_t time_total = 0;
	if (true) {
		while ((_run_limit==0 || num_runs<_run_limit) && (_time_limit==0 || time_total<_time_limit)) {
			StateSet ss(new _StateSet());
			runSimulation(s, ss);
			StateIterator sitr = StateIterator(new _StateSetIterator(*ss));
			ActionIterator aitr = _mdp->A();
			//_vi_planner->plan(sitr, aitr);
			time_total = time_in_milli()-start_time;
			num_runs++;
		}
	}
	
//	cerr << num_runs << " " << _run_limit << endl;
	
	Action a = _qtable->getBestAction(s);
//	cerr << s << "->" << a << endl;
	return a;
}

_RTDPPlanner::_RTDPPlanner(Domain domain, MDP mdp, QTable qtable,
				           Reward gamma, Reward epsilon,
				           Probability explore_epsilon, Size max_depth)
: _domain(domain), _mdp(mdp), _qtable(qtable),
  _gamma(gamma), _epsilon(epsilon),
  _run_limit(0), _time_limit(0),
  _explore_epsilon(explore_epsilon), _max_depth(max_depth),
  _vi_planner(new _VIPlanner(mdp, epsilon, gamma, _qtable)) {
//     _vi_planner->plan();
//     _qtable->print(cerr, _mdp->S(), _mdp->A());
}

_FlatRTDPPlanner::_FlatRTDPPlanner(Domain domain, MDP mdp,
					               Reward gamma, Reward epsilon,
					               Probability explore_epsilon, Size max_depth)
: _RTDPPlanner(domain, mdp, QTable(new _FQTable(domain, 0)),
               gamma, epsilon, explore_epsilon, max_depth) {

}

