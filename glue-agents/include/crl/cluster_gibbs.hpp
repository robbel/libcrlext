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

#ifndef CLUSTER_GIBBS_HPP_
#define CLUSTER_GIBBS_HPP_

#include <vector>
#include <crl/crl.hpp>
#include "crl/outcomes.hpp"
#include "crl/dpmem.hpp"

namespace crl {

class _OutcomeClusterLearner : public _Learner {
protected:
	Domain _domain;
	OutcomeTable _outcome_table;
	std::vector<Cluster> _clusters;
	_FStateTable<Size> _cluster_indices;
	DPMem _dp;
	_FActionTable<std::vector<Size> > _cluster_priors;
public:
	_OutcomeClusterLearner(const Domain& domain, const std::vector<Outcome>& outcomes);
	virtual ~_OutcomeClusterLearner() { }
	Cluster createNewCluster();
	void assignInitialClusters();
	void gibbsSweepClusters();
	void inferClusters();
	void print();
	virtual bool observe(const State& s, const Action& a, const Observation& o);
	void printClusters();
};
typedef boost::shared_ptr<_OutcomeClusterLearner> OutcomeClusterLearner;

};

#endif /* CLUSTER_GIBBS_HPP_ */
