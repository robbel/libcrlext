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

//ohhhh owww
extern int _domain_type;
extern time_t total_cluster_logp;
extern time_t count_cluster_logp;

_OutcomeClusterLearner::_OutcomeClusterLearner(const Domain& domain, const vector<Outcome>& outcomes, Probability alpha, Size m, Size t_priors, float alpha_prior, float beta_prior)
: _domain(domain), _outcomes(outcomes), _outcome_table(new _OutcomeTable(domain, _outcomes)),
  _cluster_indices(_domain, -1), _dp(new _IncrementDPMem(alpha)),
  _cluster_priors(domain, vector<Size>(_outcomes.size(), t_priors)),
  _reward_totals(new _FStateActionTable<Reward>(_domain, 0)),
  _reward_Beta_alpha(new _FStateActionTable<Reward>(_domain, alpha_prior)),
  _reward_Beta_beta(new _FStateActionTable<Reward>(_domain, beta_prior)),
  _sa_counter(new _FCounter(_domain)), _m(m) {
	//cerr << "+_OutcomeClusterLearner::_OutcomeClusterLearner" << endl;
	//assignInitialClusters();
	//cerr << "-_OutcomeClusterLearner::_OutcomeClusterLearner" << endl;
}
void _OutcomeClusterLearner::setGSLRandom(gsl_rng* gsl_random) {
	_gsl_random = gsl_random;
}

void _OutcomeClusterLearner::print() {
	_outcome_table->print();
}
bool _OutcomeClusterLearner::observe(const State& s, const Action& a, const Observation& o) {
	_last_state = s;
	_sa_counter->observe(s, a, o);
	bool learned = false;
	if (_sa_counter->getCount(s, a) <= _m)// && _sa_counter->getCount(s, a) >= _m/2)
		learned =  true;
//	cerr << "+_OutcomeClusterLearner::observe" << endl;
//	cerr << s << " x " << a << " -> " << o->getState() << endl;
	_clustered_states.insert(s);
	Index cluster_index = _cluster_indices.getValue(s);
	if (cluster_index != -1)
		_clusters[_cluster_indices.getValue(s)]->removeState(s);
	_outcome_table->observe(s, a, o);
	if (cluster_index != -1)
		_clusters[_cluster_indices.getValue(s)]->addState(s);
	//_outcome_table->print();
//	cerr << "-_OutcomeClusterLearner::observe" << endl;

	Reward old_r = _reward_totals->getValue(s, a);
	_reward_totals->setValue(s, a, old_r+o->getReward());
	
	Reward scaled_r = o->getReward()-_domain->getRewardRange().getMin();
	scaled_r /= _domain->getRewardRange().getMax()-_domain->getRewardRange().getMin();
	Reward old_alpha = _reward_Beta_alpha->getValue(s, a);
	_reward_Beta_alpha->setValue(s, a, old_alpha+scaled_r);
	Reward old_beta = _reward_Beta_beta->getValue(s, a);
	_reward_Beta_beta->setValue(s, a, old_beta+1-scaled_r);
	
//	printClusters();
	return learned;
}
void _OutcomeClusterLearner::printClusters() {
	Size num_clusters = 0;
	for (Size i=0; i<_clusters.size(); i++) {
		if (_clusters[i]->size() != 0)
			num_clusters++;
	}
//	cerr << num_clusters << " clusters" << endl;
	StateIterator sitr(new _StateIncrementIterator(_domain));
	while (sitr->hasNext()) {
		State s = sitr->next();
		if (_domain_type == 1 && s.getFactor(2) == 1)
			continue; //maze purgatory
		Index i = _cluster_indices.getValue(s);
		if (i == -1)
			cerr << "? ";
		else
			cerr << i << " ";
	}
	cerr << endl;
	/*
	for (Size i=0; i<_clusters.size(); i++) {
		if (_clusters[i]->size() == 0)
			continue;
		cerr << " " << i << ":";
		_clusters[i]->print();
		cerr << endl;
	}
	*/
/*
	cerr << "cluster dump" << endl;
	for (Size i=0; i<_clusters.size(); i++) {
		cerr << "cluster " << i << endl;
		_clusters[i]->print();
	}*/
}

Cluster _OutcomeClusterLearner::createNewCluster() {
	Cluster new_cluster(new _Cluster(_domain, _outcome_table, _cluster_priors, _gsl_random));
	return new_cluster;
}

void _OutcomeClusterLearner::gibbsSweepClusters(double temperature) {
//	cerr << "+gibbsSweepClusters" << endl;
	//StateIterator sitr(new _StateSetIterator(_clustered_states));
	StateIterator sitr(new _StateIncrementIterator(_domain));
	while (sitr->hasNext()) {
		State s = sitr->next();
		if (_domain_type == 1 && s.getFactor(2) == 1)
			continue; //maze purgatory
		//first we desample s's current cluster
		Index current_index = _cluster_indices.getValue(s);
		if (current_index != -1) {
			//cerr << "resampling cluster for " << s << "(" << current_index << ")" << endl;
			Cluster current_cluster = _clusters[current_index];
			current_cluster->removeState(s);
			_dp->undraw(current_index);
		}
		//if a new cluster is assigned to this state, this is its index
		Size new_index = _dp->peekUnseen();
		//make sure it exists
		if (new_index >= _clusters.size())
			_clusters.resize(new_index+1);
		if (!_clusters[new_index])
			_clusters[new_index] = createNewCluster();
		//store the unnormalized likelihood of each cluster in this vector
		vector<Probability> cluster_likelihoods;
		Probability max_log_p = -1*numeric_limits<Probability>::max();
//		cerr << "+resample " << s << endl;
		for (Size candidate_index=0; candidate_index<_clusters.size(); candidate_index++) {
//			cerr << " candidate = " << candidate_index << endl;
			Cluster candidate_cluster = _clusters[candidate_index];
			//there are some placeholder empty clusters that must be ignored for now
			if (candidate_cluster->size() == 0 && candidate_index != new_index) {
				cluster_likelihoods.push_back(1);
				continue;
			}
			//add and remove because we sample C_i|C_i,C_{-i}, not C_i|C_{-i}
			candidate_cluster->addState(s);
			
			
			//here is the good stuff
			
			//this is a naive and inaccurate (but fast) way
			//Probability log_p = candidate_cluster->logNoModelP(s);
			
			//this is the correct way
			Probability log_p = 0;
//			cerr << " +P(C|D)" << endl;
			for (Size i=0; i<_clusters.size(); i++) {
				Cluster c = _clusters[i];
				if (c->size() == 0)
					continue;
				log_p += c->logP();
			}
//			cerr << " -P(C|D) = " << log_p << endl;
			
			candidate_cluster->removeState(s);
			Probability dp_p = _dp->P(candidate_index);
			log_p += log(dp_p);
			//anealing step
			log_p *= 1.0/temperature;
			if (log_p > max_log_p)
				max_log_p = log_p;
			cluster_likelihoods.push_back(log_p);
		}
//		cerr << "-resample" << endl;
		//cerr << "unnormalized cluster likelihoods are:";
		Probability p_sum = 0;
		for (Size candidate_index=0; candidate_index<cluster_likelihoods.size(); candidate_index++) {
			if (cluster_likelihoods[candidate_index] == 1)
				continue;
			cluster_likelihoods[candidate_index] -= max_log_p;
			//cerr << " " << candidate_index << ":" << pow(M_E, cluster_likelihoods[candidate_index]);
			p_sum += pow(M_E, cluster_likelihoods[candidate_index]);
		}
		//cerr << endl;
		//now sample
		Index chosen_index = -1;
		Probability r = randDouble()*p_sum;
		Probability c = 0;
		for (Size candidate_index=0; candidate_index<cluster_likelihoods.size(); candidate_index++) {
			if (cluster_likelihoods[candidate_index] == 1)
				continue;
			Probability p = pow(M_E, cluster_likelihoods[candidate_index]);
			c += p;
			if (r < c) {
				chosen_index = candidate_index;
				break;
			}
		}
		//cerr << "chose cluster " << chosen_index << endl;
		_dp->draw(chosen_index);
		Cluster chosen_cluster = _clusters[chosen_index];
		chosen_cluster->addState(s);
		_cluster_indices.setValue(s, chosen_index);
	}
//	if (count_cluster_logp != 0) {
//		double time_per_cluster = 1.0*total_cluster_logp/count_cluster_logp;
//		cerr << "avg = " << time_per_cluster << " with count = " << count_cluster_logp << endl;
//		cerr << _domain->getNumStates()*_domain->getNumStates()*_domain->getNumStates() << endl;
//		total_cluster_logp = 0;
//		count_cluster_logp = 0;
//	}
	//printClusters();
//	cerr << "-gibbsSweepClusters" << endl;
}


vector<MDP> _OutcomeClusterLearner::sampleMDPs(Size k, Size burn, Size spacing, bool anneal) {
//	cerr << "+_OutcomeClusterLearner::sampleMDPs" << endl;
	vector<MDP> mdps;

	Probability max_ll = 1;
	double temperature = 1;
	if (anneal) {
		temperature = (burn+k*spacing)*.1;
	}
//	cerr << "burn" << endl;
	for (Size i=0; i<burn; i++) {
//		cerr << "sweep " << i << endl;
		gibbsSweepClusters(temperature);
		if (anneal) temperature -= .1;
	}
	for (Size j=0; j<k; j++) {
		for (Size i=0; j!=0 && i<spacing; i++) {
			gibbsSweepClusters(temperature);
			if (anneal) temperature -= .1;
		}
			
		//if (0 && _domain_type == 1)
			//printClusters();

		Probability mdp_ll = 0;
		for (Size i=0; i<_clusters.size(); i++) {
			if (_clusters[i]->size() == 0)
				continue;
			mdp_ll += _clusters[i]->logP();
		}
		mdp_ll += _dp->logP();
		

		ClusterMDP mdp(new _ClusterMDP(_domain, _outcomes, _clusters, _cluster_indices, mdp_ll));
		mdp->setRewardBeta(_reward_Beta_alpha, _reward_Beta_beta);
		mdp->setRewardTotals(_reward_totals, _sa_counter);
		mdp->setGSLRandom(_gsl_random);
//		mdp->setRewardTotals(_reward_totals);
//		mdp->setCounter(_sa_counter);
		if (max_ll == 1 || mdp_ll > max_ll) {
			max_ll = mdp_ll;
			mdps.insert(mdps.begin(), mdp);
		}
		else
			mdps.push_back(mdp);
//		mdp->printXML(cerr);
	}
//	printClusters();
//	cerr << endl;
//	cerr << "-_OutcomeClusterLearner::sampleMDPs" << endl;
	return mdps;
}
