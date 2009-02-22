#include <cmath>
#include <iostream>
#include "cpputil.hpp"
#include "crl/uct.hpp"

using namespace std;
using namespace cpputil;
using namespace crl;

#define LOGUCT 0

// -------------------------------------------------------------------------------
// _ICTPlanner

Action _IUCTPlanner::getAction(const State& s) {
	// (*) initialiset
	Size num_runs = 0;
	time_t start_time = time_in_milli();
	time_t time_total = 0;
	if (_clear_tree) {
		ClearTree();
	}

	// (*) loop over simulations
	while ((_run_limit==0 || num_runs<_run_limit) && //0 for limit means unlimited
          (_time_limit==0 || time_total<_time_limit)) {

		runSimulation(s);

		time_total = time_in_milli()-start_time;
		num_runs++;
	}

	// (*) return the best action without bonuses
	return _qtable->getBestAction(s);
}

void _IUCTPlanner::runSimulation(const State& s) {
	_CTraceList trace; // trace
	int steps=0;
	Reward start_metric = 0;	// Reward metric when we start (we start with 0 here and add up everything encountered on the trace  during sampling).
	Reward current_metric = 0;	// Reward metric when we start (we start with 0 here and add up everything encountered on the trace  during sampling).
	State current_state = s;

	// Sampling one trajectory until the goal state is reached.
	bool goal_is_reached = false;
	do{
//cerr << "step=" << steps << " current state is: " << current_state << endl;
		ActionIterator curr_actions_iter = _mdp->A(current_state);
		Size selected_action;
		Action action;
		if (!_qtable->inTree(current_state.getIndex())) { // If not in the tree just pick up one action heuristically.
			action = GetHuristicAction();
			selected_action = action.getIndex();
		} else {
			action =_qtable->GetUCTAction(current_state.getIndex(), selected_action);
		}
//cerr << "Performing action " << action << endl;
		State new_state;
		goal_is_reached = !(_mdp->A(current_state)->hasNext());
		try{
			new_state = _mdp->T(current_state,action)->sample();
		}
		catch (DistributionException e) {
			// sampling throws an exception if all next states have probability 0.
			goal_is_reached = true;
		}
		Reward step_reward = _mdp->R(current_state,action);

		if (!goal_is_reached) {
			CTraceData temp(new _CTraceData(current_state.getIndex(), selected_action, new_state.getIndex(), step_reward));
			trace.push_back(temp);
		}

		Reward new_metric = current_metric + step_reward;
		current_metric = new_metric;
		if (_fullTree && !goal_is_reached) {
			// Add immediately so it can influence exploration within this episode.
			_qtable->Visit(current_state.getIndex(), selected_action);
		}

		current_state = new_state;
//cerr << "new state is: " << new_state << endl;
	}while( (steps++ < _maxSteps || _maxSteps == 0) && !goal_is_reached);

	Reward cumulative_reward = current_metric - start_metric; // Cumulative reward.
	if (!goal_is_reached) {
//		cerr << "the goal state has not been reached, penalising the trace !!!" << endl;
		cumulative_reward += -100; // if the steps limit exceeded then penalise this trajectory.
	}

	_qtable->DoBackups(trace, cumulative_reward, _fullTree);
}
