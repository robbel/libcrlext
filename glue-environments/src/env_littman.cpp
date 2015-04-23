/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth

    This file is part of CRL:Glue-environments.

    CRL:RL-environments is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CRL:RL-environments is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CRL:RL-environments.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <strxml.hpp>
#include <string.h>
#include <rlglue/Environment_common.h>
#include <rlglue/RL_glue.h>
#include <rlgnmenv.h>
#include <cpputil.hpp>
#include <crl/crl.hpp>
#include <crl/flat_tables.hpp>
#include "crl/glue_env.hpp"

using namespace std;
using namespace cpputil;
using namespace crl;

int num_links;
int num_types;
string link_assignments;

/*
<LittmanChain>
	<ClusterAssignments count="5">01210</ClusterAssignments>
	<Clusters count="3">
		<Cluster index="0">
			<Action index="0">
				<left>.9</left>
				<right>.1</right>
				<stay>0</stay>
			</Action>
			<Action index="1">
				<left>.1</left>
				<right>.9</right>
				<stay>0</stay>
			</Action>
			<Action index="2">
				<left>0</left>
				<right>0</right>
				<stay>1</stay>
			</Action>
			<Action index="3">
				<left>.4</left>
				<right>.4</right>
				<stay>.2</stay>
			</Action>
		</Cluster>
		<Cluster index="1">
			<Action index="0">
				<left>0</left>
				<right>0</right>
				<stay>1</stay>
			</Action>
			<Action index="1">
				<left>0</left>
				<right>0</right>
				<stay>1</stay>
			</Action>
			<Action index="2">
				<left>0</left>
				<right>0</right>
				<stay>1</stay>
			</Action>
			<Action index="3">
				<left>.4</left>
				<right>.4</right>
				<stay>.2</stay>
			</Action>
		</Cluster>
		<Cluster index="2">
			<Action index="0">
				<left>.3</left>
				<right>.5</right>
				<stay>.2</stay>
			</Action>
			<Action index="1">
				<left>.5</left>
				<right>.3</right>
				<stay>.2</stay>
			</Action>
			<Action index="2">
				<left>0</left>
				<right>0</right>
				<stay>1</stay>
			</Action>
			<Action index="3">
				<left>.4</left>
				<right>.4</right>
				<stay>.2</stay>
			</Action>
		</Cluster>
	</Clusters>
</LittmanChain>

 */

class Cluster {
	class Distribution {
	public:
		Probability left;
		Probability right;
		Probability stay;
		Distribution() {
			left = 0; right = 0; stay = 1;
		}
	};
	vector<Distribution> _outcome_distributions;
public:
	void read(XMLObject xobj) {
		_outcome_distributions.resize(4);
		for (int i=0; i<xobj.size(); i++) {
			XMLObject actionObj = xobj[i];
			if (actionObj.getName() != "Action")
				continue;
			Size action_index = atoi(actionObj("index").c_str());
			_outcome_distributions[action_index].left = atof(actionObj["left"].getText().c_str());
			_outcome_distributions[action_index].right = atof(actionObj["right"].getText().c_str());
			_outcome_distributions[action_index].stay = atof(actionObj["stay"].getText().c_str());
		}
	}
	Probability left(const Action& a) const {
		return _outcome_distributions[a.getIndex()].left;
	}
	Probability right(const Action& a) const {
		return _outcome_distributions[a.getIndex()].right;
	}
	Probability stay(const Action& a) const {
		return _outcome_distributions[a.getIndex()].stay;
	}
};

vector<Cluster> clusters;

class _LittmanChainEnv : public _Environment {
protected:
	Domain _domain;
	State _current;
	vector<int> _state_types;
public:
	_LittmanChainEnv(const Domain& domain);
	virtual ~_LittmanChainEnv() { }

	virtual State begin();
	virtual bool isTerminated();
	virtual Observation getObservation(const Action& a);
};
typedef boost::shared_ptr<_LittmanChainEnv> LittmanChainEnv;

_LittmanChainEnv::_LittmanChainEnv(const Domain& domain)
: _domain(domain) {
	for (int i=0; i<num_links; i++) {
		char assignment = link_assignments[i];
		if (assignment == 'T')
			_state_types.push_back(-1);
		else
			_state_types.push_back(assignment-'0');
//		cerr << assignment;
	}
//	cerr << endl;
}

State _LittmanChainEnv::begin() {
	_current = State(_domain);
	_current.setFactor(0, 0);
	return _current;
}

bool _LittmanChainEnv::isTerminated() {
	return _state_types[_current.getFactor(0)] == -1;
}

Observation _LittmanChainEnv::getObservation(const Action& a) {


//	cerr << _current << " x " << a << " -> ";
	Factor link = _current.getFactor(0);
//	if (link == 5)
//		cerr << 5;
	Size cluster_index = _state_types[link];
	Cluster& c = clusters[cluster_index];
	Probability left = c.left(a);
	Probability right = c.right(a);
	Probability stay = c.stay(a);

	Probability r = randDouble();
	if (r < left) {
		if (link > 0)
			link--;
	}
	else if (r < left+right) {
		link++;
	}
	else if (r < left+right+stay) {

	}
	_current.setFactor(0, link);
//	cerr << _current << endl;
	Observation o(new _Observation(_current, -1));
	return o;
}


Domain crl::getCRLEnvironmentDomain() {
	Domain domain(new _Domain());
	domain->addStateFactor(0, num_links-1);
	domain->addActionFactor(0, 3);
	domain->setRewardRange(-1, 0);
	return domain;
}

Environment crl::getCRLEnvironment(Domain domain) {
	LittmanChainEnv env(new _LittmanChainEnv(domain));
	return env;
}

char paramBuf[256];
const char* env_message(const char* inMessage) {
	if (!strcmp(inMessage, "id"))
		return (char*)"littman-chain";
	else if (!strcmp(inMessage, "param"))
		return paramBuf;
	else if (!strcmp(inMessage, "version"))
		return (char*)"1";
	else if (!strncmp(inMessage, "seed", 4)) {
		long seed = atoi(inMessage+5);
		srand(seed);
	}
	return (char*)"";
 }
/*
<Chain>
	<ClusterAssignments count="6">10101T</ClusterAssignments>
	<Clusters count="2">
		<Cluster index="0">.7</Cluster>
		<Cluster index="1">.3</Cluster>
	</Clusters>
</Chain>
 */
int main(int argc, char** argv) {
	if (argc != 2) {
		cerr << "Usage: " << argv[0] << " <config>" << endl;
		return 1;
	}
	ifstream is(argv[1]);
	XMLObject xobj(is);
	XMLObject clusterAssignments = xobj["ClusterAssignments"];
	num_links = atoi(clusterAssignments("count").c_str());
	link_assignments = clusterAssignments.getText();

	XMLObject clustersObj = xobj["Clusters"];
	Size numClusters = atoi(clustersObj("count").c_str());
	clusters.resize(numClusters);
	for (int i=0; i<clustersObj.size(); i++) {
		XMLObject clusterObj = clustersObj[i];
		if (clusterObj.getName() != "Cluster")
			continue;
		int index = atoi(clusterObj("index").c_str());
		clusters[index].read(clusterObj);
	}

	paramBuf[0] = '\0';

	glue_main_env(0, 0);

	return 0;
}

