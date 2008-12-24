#include <iostream>
#include "cpputil.hpp"
#include "crl/rtdp.hpp"

using namespace std;
using namespace cpputil;
using namespace crl;

Reward _RTDPPlanner::runSimulation(const State& s, StateSet ss, Size depth) {
	if (depth >= _max_depth)
		return 0;
		
//	ss->insert(s);

	Index count = _s_counts->getValue(s);
	if (count >= _m)
		return 0;
//	cerr << count << endl;
		
	Action a = _qtable->getBestAction(s);
	if (randDouble() < _explore_epsilon) {
		a = Action(_domain, random()%_domain->getNumActions());
	}
	
	Reward error = 0;
	
	try {
		State n = _mdp->T(s, a)->sample();
		
		Reward next_error = runSimulation(n, ss, depth+1);
		
		ActionIterator aitr = _mdp->A();
		error = _vi_planner->backupState(s, aitr);
		_s_counts->setValue(s, count+1);
		if (next_error > error)
			error = next_error;
			
	}
	catch (DistributionException e) {
//		cerr << "reached terminal state" << endl;
	}
	return error;
}

Action _RTDPPlanner::getAction(const State& s) {
	Size num_runs = 0;
	time_t start_time = time_in_milli();
	time_t time_total = 0;
	
//	if (_s_counts->getValue(s) < _m) {
	while (_s_counts->getValue(s) < _m && (_run_limit==0 || num_runs<_run_limit) && (_time_limit==0 || time_total<_time_limit)) {
		
		StateSet ss(new _StateSet());
		Reward error = runSimulation(s, ss);
		
//		StateIterator sitr = StateIterator(new _StateSetIterator(*ss));
//		ActionIterator aitr = _mdp->A();
		//_vi_planner->plan(sitr, aitr);
		time_total = time_in_milli()-start_time;
		num_runs++;
		
		if (error < _epsilon) {
//			break;
		}
	}
	
	//always back up the queried state
	ActionIterator aitr = _mdp->A();
	_vi_planner->backupState(s, aitr);
//	}
	
//	cerr << num_runs << " " << _run_limit << endl;
	
	Action a = _qtable->getBestAction(s);
//	cerr << s << "->" << a << endl;
	return a;
}

_RTDPPlanner::_RTDPPlanner(Domain domain, MDP mdp, QTable qtable,
						   SCountTable s_counts,
				           Reward gamma, Reward epsilon, Index m,
				           Probability explore_epsilon, Size max_depth)
: _domain(domain), _mdp(mdp), _qtable(qtable), _s_counts(s_counts),
  _gamma(gamma), _epsilon(epsilon), _m(m),
  _run_limit(0), _time_limit(0),
  _explore_epsilon(explore_epsilon), _max_depth(max_depth),
  _vi_planner(new _VIPlanner(mdp, epsilon, gamma, _qtable)) {
//     _vi_planner->plan();
//     _qtable->print(cerr, _mdp->S(), _mdp->A());
}

_FlatRTDPPlanner::_FlatRTDPPlanner(Domain domain, MDP mdp,
					               Reward gamma, Reward epsilon, Index m,
					               Probability explore_epsilon, Size max_depth)
: _RTDPPlanner(domain, mdp, QTable(new _FQTable(domain, 0)),
			   SCountTable(new _FStateTable<Index>(domain, 0)),
               gamma, epsilon, m, explore_epsilon, max_depth) {

}

