#ifndef RTDP_HPP_
#define RTDP_HPP_

#include <boost/shared_ptr.hpp>
#include <sys/time.h>

#include "crl/crl.hpp"
#include "crl/vi.hpp"
#include "crl/fdomain.hpp"
#include "crl/hdomain.hpp"


namespace crl {

/**
 * A planner that uses RTDP
 */
class _RTDPPlanner : public _Planner {
protected:
	/**
	 * The domain describes what valid states and actions are.
	 */
	Domain _domain;
	/**
	 * The MDP to plan with
	 */
	MDP _mdp;
	/**
	 * The q-table to store values in
	 */
	QTable _qtable;
	/**
	 * A table to keep track of how many times a state has gone through a roll-out
	 */
	SCountTable _s_counts;
	
	/**
	 * discount rate
	 */
	Reward _gamma;
	/**
	 * unused at the moment
	 */
	Reward _epsilon;
	
	/**
	 * Max number of times a state can be in a roll-out
	 */
	Index _m;
	
	/**
	 * Max number of roll-outs per call to getAction
	 */
	Size _run_limit;
	/**
	 * Max number milliseconds per call to getAction
	 */
	time_t _time_limit;
	
	/**
	 * Likelihood that a roll-out explores a random action
	 */
	Probability _explore_epsilon;
	/**
	 * Max depth for a roll-out
	 */
	Size _max_depth;
	
	/**
	 * used for its ability to perform Bellman backups
	 */
	VIPlanner _vi_planner;
	
	/**
	 * Perform a simulation/roll-out starting at s. Returns the biggest
	 * Bellman residual.
	 */
	Reward runSimulation(const State& s, Size depth=0);
	
	_RTDPPlanner(Domain domain, MDP mdp, QTable qtable, SCountTable s_counts,
	             Reward gamma, Reward epsilon, Index m,
	             Probability explore_epsilon, Size max_depth);
public:
	virtual ~_RTDPPlanner() { }
	
	void setRunLimit(Size run_limit) {_run_limit=run_limit;}
	void setTimeLimit(time_t time_limit) {_time_limit=time_limit;}
	
	/**
	 * from _Planner
	 */
	virtual Action getAction(const State& s);
};
typedef boost::shared_ptr<_RTDPPlanner> RTDPPlanner;

/**
 * This is a subclass that provides the necessary data structures. This
 * version of RTDPPlanner provides a flat q-table and state count table.
 */
class _FlatRTDPPlanner : public _RTDPPlanner {
public:
	_FlatRTDPPlanner(Domain domain, MDP mdp,
		             Reward gamma, Reward epsilon, Index m,
		             Probability explore_epsilon, Size max_depth);
};

}

#endif /*RTDP_HPP_*/
