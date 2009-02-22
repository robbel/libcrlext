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

#include <math.h>
#include <iostream>
#include <cpputil.hpp>
#include "crl/dpmem.hpp"
#include "crl/outcomes.hpp"
#include "crl/cluster_gibbs.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

_OutcomeClusterLearner::_OutcomeClusterLearner(const Domain& domain, const vector<Outcome>& outcomes)
: _domain(domain), _outcome_table(new _OutcomeTable(domain, outcomes)),
  _cluster_indices(_domain, 0), _dp(new _IncrementDPMem(1)) {
	cerr << "+_OutcomeClusterLearner::_OutcomeClusterLearner" << endl;
	assignInitialClusters();
	cerr << "-_OutcomeClusterLearner::_OutcomeClusterLearner" << endl;
}
void _OutcomeClusterLearner::inferClusters() {
	gibbsSweepClusters();
}
void _OutcomeClusterLearner::print() {
	_outcome_table->print();
}
bool _OutcomeClusterLearner::observe(const State& s, const Action& a, const Observation& o) {
	//cerr << s << " x " << a << " -> " << o->getState() << endl;
	Cluster c = _clusters[_cluster_indices.getValue(s)];
	c->removeState(s);
	_outcome_table->observe(s, a, o);
	c->addState(s);
	//_outcome_table->print();
	return true;
}
void _OutcomeClusterLearner::printClusters() {
	cerr << "cluster dump" << endl;
	StateIterator sitr(new _StateIncrementIterator(_domain));
	while (sitr->hasNext()) {
		State s = sitr->next();
		cerr << _cluster_indices.getValue(s);
	}
	cerr << endl;
	for (Size i=0; i<_clusters.size(); i++) {
		cerr << "cluster " << i << endl;
		_clusters[i]->print();
	}
}
void _OutcomeClusterLearner::assignInitialClusters() {
	cerr << "+assignInitialClusters" << endl;
	StateIterator sitr(new _StateIncrementIterator(_domain));
	while (sitr->hasNext()) {
		State s = sitr->next();
		cerr << "assigning cluster for " << s << endl;
		Size new_index = _dp->draw();
		if (new_index >= _clusters.size())
			_clusters.resize(new_index+1);
		if (!_clusters[new_index]) {
			Cluster new_cluster(new _Cluster(_domain, _outcome_table));
			_clusters[new_index] = new_cluster;
		}
		_cluster_indices.setValue(s, new_index);
		_clusters[new_index]->addState(s);
	}
	cerr << "-assignInitialClusters" << endl;
}

void _OutcomeClusterLearner::gibbsSweepClusters() {
	cerr << "+gibbsSweepClusters" << endl;
	StateIterator sitr(new _StateIncrementIterator(_domain));
	while (sitr->hasNext()) {
		State s = sitr->next();
		cerr << "resampling cluster for " << s << endl;
		//first we desample s's current cluster
		Size current_index = _cluster_indices.getValue(s);
		Cluster current_cluster = _clusters[current_index];
		current_cluster->removeState(s);
		_dp->undraw(current_index);
		//if a new cluster is assigned to this state, this is its index
		Size new_index = _dp->peekUnseen();
		//make sure it exists
		if (new_index >= _clusters.size())
			_clusters.resize(new_index+1);
		if (!_clusters[new_index])
			_clusters[new_index] = Cluster(new _Cluster(_domain, _outcome_table));
		//store the unnormalized likelihood of each cluster in this vector
		vector<Probability> cluster_likelihoods;
		Probability max_log_p = -1*numeric_limits<Probability>::max();
		for (Size candidate_index=0; candidate_index<_clusters.size(); candidate_index++) {
			Cluster candidate_cluster = _clusters[candidate_index];
			//there are some placeholder empty clusters that must be ignored for now
			if (candidate_cluster->size() == 0 && candidate_index != new_index) {
				cluster_likelihoods.push_back(1);
				continue;
			}
			//add and remove because we sample C_i|C_i,C_{-i}, not C_i|C_{-i}
			candidate_cluster->addState(s);
			cerr << "about to find logP" << endl;
			Probability log_p = candidate_cluster->logP(s);
			cerr << " cluster " << candidate_index << " has weight " << log_p << endl;
			candidate_cluster->removeState(s);
			log_p += log(_dp->P(candidate_index));
			if (log_p > max_log_p)
				max_log_p = log_p;
			cerr << "  with the DP, weight " << log_p << endl;
			cluster_likelihoods.push_back(log_p);
		}
		Probability log_sum = 0;
		for (Size candidate_index=0; candidate_index<cluster_likelihoods.size(); candidate_index++) {
			if (cluster_likelihoods[candidate_index] == 1)
				continue;
			cluster_likelihoods[candidate_index] -= max_log_p;
			log_sum += pow(M_E, cluster_likelihoods[candidate_index]);
		}
		Probability normInv = 1.0/log_sum;
		//now sample
		Size chosen_index = 0;
		Probability r = randDouble();
		Probability c = 0;
		for (Size candidate_index=0; candidate_index<cluster_likelihoods.size(); candidate_index++) {
			if (cluster_likelihoods[candidate_index] == 1)
				continue;
			Probability p = pow(M_E, cluster_likelihoods[candidate_index]);
			c += normInv*p;
			if (r < c) {
				chosen_index = candidate_index;
				break;
			}
		}
		_dp->draw(chosen_index);
		Cluster chosen_cluster = _clusters[chosen_index];
		chosen_cluster->addState(s);
	}
	cerr << "-gibbsSweepClusters" << endl;
}
