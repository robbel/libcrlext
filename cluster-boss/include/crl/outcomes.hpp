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

#ifndef OUTCOMES_HPP_
#define OUTCOMES_HPP_

#include <vector>
#include <boost/shared_ptr.hpp>
#include <gsl/gsl_rng.h>

#include <crl/crl.hpp>
#include <crl/hash_tables.hpp>
#include <crl/flat_tables.hpp>

namespace crl {

typedef boost::shared_ptr<_FStateActionTable<Reward> > FStateActionRewardTable;

class _Outcome {
public:
	virtual bool match(const State& s, const State& sp) = 0;
	virtual State apply(const State& s) = 0;
	virtual ~_Outcome() { }
};
typedef boost::shared_ptr<_Outcome> Outcome;

class _StepOutcome : public _Outcome {
protected:
	Domain _domain;
	const std::vector<int> _deltas;
	bool _range_barrier;
public:
	_StepOutcome(Domain domain, const std::vector<int>& deltas, bool range_barrier);
	virtual ~_StepOutcome() { }
	virtual bool match(const State& s, const State& sp);
	virtual State apply(const State& s);
};
typedef boost::shared_ptr<_StepOutcome> StepOutcome;

class _FixedOutcome : public _Outcome {
protected:
	const State _s;
public:
	_FixedOutcome(const State& s);
	virtual ~_FixedOutcome() { }
	virtual bool match(const State& s, const State& sp);
	virtual State apply(const State& s);
};
typedef boost::shared_ptr<_FixedOutcome> FixedOutcome;

class _TerminalOutcome : public _Outcome {
public:
	virtual ~_TerminalOutcome() { }
	virtual bool match(const State& s, const State& sp);
	virtual State apply(const State& s);
};
typedef boost::shared_ptr<_TerminalOutcome> TerminalOutcome;

class _OutcomeTable : _Learner {
protected:
	Domain _domain;
	std::vector<Outcome> _outcomes;
	_HStateActionTable<std::vector<Size> > _outcomeCounts;
	_HStateActionTable<std::vector<Probability> > _outcomeDistributions;
public:
	_OutcomeTable(const Domain& domain, const std::vector<Outcome>& outcomes);
	virtual ~_OutcomeTable() { }
	virtual bool observe(const State& s, const Action& a, const Observation& o);

	std::vector<Size>& getOutcomeCounts(const State& s, const Action& a) {
		return _outcomeCounts.getValue(s, a);
	}
	Outcome getOutcome(Size index) {
		return _outcomes[index];
	}
	Index getOutcomeIndex(Outcome o) {
		for (Size i=0; i<_outcomes.size(); i++)
			if (o == _outcomes[i])
				return i;
		return -1;
	}
	Size numOutcomes() {return _outcomes.size();}

	void print();
};
typedef boost::shared_ptr<_OutcomeTable> OutcomeTable;

class _Cluster {
protected:
	Domain _domain;
	OutcomeTable _outcome_table;
	std::set<State> _states;
	_FActionTable<std::vector<Size> > _outcome_priors;
	_FActionTable<std::vector<Size> > _outcome_counts;
	_FActionTable<Size> _outcome_totals;
	_FActionTable<std::vector<Probability> > _outcome_probs;
	_FActionTable<std::vector<Probability> > _outcome_probs_no_model;
	Size _num_states;
	gsl_rng* _gsl_random;
	Probability _log_p;
public:
	_Cluster(const Domain& domain, OutcomeTable outcome_table, gsl_rng* gsl_random);
	_Cluster(const Domain& domain, OutcomeTable outcome_table, _FActionTable<std::vector<Size> > outcome_priors, gsl_rng* gsl_random);
	virtual ~_Cluster() { }
	virtual void setGSLRandom(gsl_rng* gsl_random);
	void addState(const State& s);
	void removeState(const State& s);
	void calcProbs();
	void calcProbs(const Action& a);
	Size size();
	std::vector<Size>& getCounts(const Action& a);
	Size getTotal(const Action& a) {return _outcome_totals.getValue(a);}
	virtual Probability P(const Action& a, const Outcome& o);
	virtual Probability P(const Action& a, Size index);
	virtual Probability noModelP(const Action& a, const Outcome& o);
	virtual Probability logNoModelP(const State& s);

	virtual Probability calcLogP();
	virtual Probability logP();

	void print();
};
typedef boost::shared_ptr<_Cluster> Cluster;

class _ClusterMDP;
typedef boost::shared_ptr<_ClusterMDP> ClusterMDP;
class _ClusterMDP : public _MDP {
protected:
	Domain _domain;
	std::vector<Outcome> _outcomes;
	_FStateTable<Cluster> _clusters;
	std::vector<Cluster> _cluster_vec;
	_FStateActionTable<Reward> _rewards;
	FStateActionRewardTable _reward_Beta_alpha;
	FStateActionRewardTable _reward_Beta_beta;
	FStateActionRewardTable _reward_totals;
	FCounter _sa_counter;
	_FStateActionTable<FStateDistribution> _T_map;
	gsl_rng* _gsl_random;
	Probability _log_p;
	bool _use_Beta;
	Size _m;
public:
	_ClusterMDP(const Domain& domain, std::vector<Outcome> outcomes, std::vector<Cluster>& cluster_vec, _FStateTable<Index>& cluster_indices, Probability log_p);
	virtual void setRewardBeta(FStateActionRewardTable reward_Beta_alpha, FStateActionRewardTable reward_Beta_beta);
	virtual void setRewardTotals(FStateActionRewardTable reward_totals, FCounter sa_counter);
	virtual void setUseBeta(bool use_Beta) {_use_Beta = use_Beta;}
	virtual void setRmaxM(Size m) {_m = m;}
	virtual bool isKnown(const State& s, const Action& a);
	virtual void setGSLRandom(gsl_rng* gsl_random) {_gsl_random = gsl_random;}
	Outcome getOutcome(Size index) {return _outcomes[index];}
	//virtual void setCounter(FCounter sa_counter) {_sa_counter=sa_counter;}
	virtual StateIterator S();
	virtual StateIterator predecessors(const State& s);
	virtual ActionIterator A();
	virtual ActionIterator A(const State& s);
	virtual StateDistribution T(const State& s, const Action& a);
	virtual Reward R(const State& s, const Action& a);
	virtual void printXML(std::ostream& os);
	Cluster getCluster(const State& s) {return _clusters.getValue(s);}
	Probability logP();
};
typedef boost::shared_ptr<_ClusterMDP> ClusterMDP;
inline ClusterMDP getClusterMDP(MDP mdp) {
	return boost::static_pointer_cast<_ClusterMDP>(mdp);
}

};

#endif /* OUTCOMES_HPP_ */
