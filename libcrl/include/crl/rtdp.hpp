#ifndef RTDP_HPP_
#define RTDP_HPP_

#include <boost/shared_ptr.hpp>
#include <sys/time.h>

#include "crl/crl.hpp"
#include "crl/vi.hpp"
#include "crl/fdomain.hpp"
#include "crl/hdomain.hpp"


namespace crl {

class _RTDPPlanner : public _Planner {
protected:
	Domain _domain;
	MDP _mdp;
	QTable _qtable;
	
	Reward _gamma;
	Reward _epsilon;
	
	Size _run_limit;
	time_t _time_limit;
	
	Probability _explore_epsilon;
	Size _max_depth;
	
	VIPlanner _vi_planner;
	
	void runSimulation(const State& s, StateSet ss, Size depth=0);
	
	
	_RTDPPlanner(Domain domain, MDP mdp, QTable qtable,
	             Reward gamma, Reward epsilon,
	             Probability explore_epsilon, Size max_depth);
public:
	virtual ~_RTDPPlanner() { }
	
	void setRunLimit(Size run_limit) {_run_limit=run_limit;}
	void setTimeLimit(time_t time_limit) {_time_limit=time_limit;}
	
	virtual Action getAction(const State& s);
};
typedef boost::shared_ptr<_RTDPPlanner> RTDPPlanner;

class _FlatRTDPPlanner : public _RTDPPlanner {
public:
	_FlatRTDPPlanner(Domain domain, MDP mdp,
		             Reward gamma, Reward epsilon,
		             Probability explore_epsilon, Size max_depth);
};

}

#endif /*RTDP_HPP_*/
