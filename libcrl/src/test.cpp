/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth

    This file is part of CRL.

    CRL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CRL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CRL.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include "crl/crl.hpp"
#include "crl/fdomain.hpp"
#include "crl/hdomain.hpp"
#include "crl/vi.hpp"
#include "crl/uct.hpp"
#include "crl/ps.hpp"
#include "crl/sparse_sampling.hpp"
#include "crl/mdomain.hpp"
#include "crl/environment.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

void testCount(Domain domain) {
	FCounter counter(new _FCounter(domain));
	for (int i=0; i<1000; i++) {
		State s(domain, random()%domain->getNumStates());
		Action a(domain, random()%domain->getNumActions());
		State n(domain, random()%domain->getNumStates());
//		cout << s << ", " << a << " -> " << n << endl;
		Observation o(new _Observation(n, 0));
		counter->observe(s, a, o);
	}
	_StateIncrementIterator sitr(domain);
	_ActionIncrementIterator aitr(domain);
	_StateIncrementIterator nitr(domain);
	while (sitr.hasNext()) {
		State s = sitr.next();
		aitr.reset();
		while (aitr.hasNext()) {
			Action a = aitr.next();
			nitr.reset();
			cout << s << ", " << a << " : " << counter->getCount(s, a) << endl;
			while (nitr.hasNext()) {
				State n = nitr.next();
				cout << " " << n << " : " << counter->getCount(s, a, n) << endl;
			}
		}
	}
}

void testSTL() {
	Domain domain(new _Domain());
	domain->addStateFactor(0, 4);
	domain->addActionFactor(0, 4);
	domain->setRewardRange(0, 1);

	State s1(domain, 0);
	State s2(domain, 1);

	StateSet ss(new _StateSet());
	ss->insert(s1);
	ss->insert(s2);
	StateSetIterator itr(new _StateSetIterator(*ss));
	while (itr->hasNext())
		cout << itr->next() << endl;
}

MDP makeMDP(Domain domain) {

	FMDP mdp(new _FMDP(domain));

//	time_t start_time = time_in_milli();
	for (Size state_index=0; state_index<domain->getNumStates(); state_index++) {
		State s(domain, state_index);
		for (Size action_index=0; action_index<domain->getNumActions(); action_index++) {
			Action a(domain, action_index);
			mdp->setR(s, a, randDouble());
			Probability total = 0;
			std::vector<Probability> probs(domain->getNumStates());
			for (Size next_index=0; next_index<domain->getNumStates(); next_index++) {
				Probability prob = randDouble();
				total += prob;
				probs[next_index] = prob;
			}
			for (Size next_index=0; next_index<domain->getNumStates(); next_index++) {
				State s_next(domain, next_index);
				Probability prob = probs[next_index]/total;
				mdp->setT(s, a, s_next, prob);
			}
		}
	}
//	time_t end_time = time_in_milli();
//	cout << "created mdp in " << end_time - start_time << "ms" << endl;

	return mdp;
}
MDP makeTerminatingMDP(Domain domain) {

	FMDP mdp = boost::shared_polymorphic_downcast<_FMDP>(makeMDP(domain));
	State ts(domain, domain->getNumStates()-1);
	for (Size action_index=0; action_index<domain->getNumActions(); action_index++) {
		Action a(domain, action_index);
		mdp->clear(ts, a);
	}
	return mdp;
}

Action testVI(MDP mdp, Domain domain) {
	long start_time = time_in_milli();
	cout << "FVI" << endl;
	VIPlanner planner(new _FactoredVIPlanner(domain, mdp, .0001, .9));
	int count = planner->plan();
	long end_time = time_in_milli();
	QTable qtable = planner->getQTable();
	State s(domain, 0);
	cout << " V(" << s << ") = " << qtable->getV(s) << endl;
	Action a = planner->getAction(s);
	cout << " best action = " << a << endl;
	cout << "FVI finished " << count << " iterations in " << end_time - start_time << endl;
	return a;
}

Action testPS(MDP mdp, Domain domain) {
	long start_time = time_in_milli();
	cout << "FPS" << endl;
	State s(domain, 0);
	PSPlanner planner(new _FlatPSPlanner(domain, mdp, .0001, .9));
	planner->insert(s, 1);
	int count = planner->sweep();
	long end_time = time_in_milli();
	QTable qtable = planner->getQTable();
	cout << " V(" << s << ") = " << qtable->getV(s) << endl;
	Action a = planner->getAction(s);
	cout << " best action = " << a << endl;
	cout << "FPS finished " << count << " updates in " << end_time - start_time << endl;
	return a;
}

Action testHVI(MDP mdp, Domain domain) {
	long start_time = time_in_milli();
	cout << "HVI" << endl;
	VIPlanner planner(new _HashedVIPlanner(domain, mdp, .0001, .9));
	int count = planner->plan();
	long end_time = time_in_milli();
	QTable qtable = planner->getQTable();
	State s(domain, 0);
	cout << " V(" << s << ") = " << qtable->getV(s) << endl;
	Action a = planner->getAction(s);
	cout << " best action = " << a << endl;
	cout << "HVI finished " << count << " iterations in " << end_time - start_time << endl;
	return a;
}

Action testUCT(MDP mdp, Domain domain) {
	cout << "UCT" << endl;
//	long start_time = time_in_milli();
	UCTPlanner planner(new _FlatUCTPlanner(domain, mdp, 1, 1, false, true, 0));
	planner->setRunLimit(-1);
	planner->setTimeLimit(5000);
	planner->setConfidenceCoeff(1);
	State s(domain, 0);
	Action a = planner->getAction(s);
//	long end_time = time_in_milli();
	//QTable qtable = planner->getQTable();
	//cout << "V(" << s << ") = " << qtable->getV(s) << endl;
	cout << " best action = " << a << endl;
//	cout << "UCT finished in " << end_time - start_time << endl;
	return a;
}

void testSS(MDP mdp, Domain domain) {
	long start_time = time_in_milli();
	SparseSamplingPlanner planner(new _SparseSamplingPlanner(mdp, 0.9, 2, 6));
	State s(domain, 0);
	planner->getAction(s);
	long end_time = time_in_milli();
	cout << "sparse sampling finished in " << end_time - start_time << endl;
}

void testExperiment(Domain domain) {
	MDP mdp = makeTerminatingMDP(domain);
	State s(domain, 0);

	VIPlanner planner1(new _FactoredVIPlanner(domain, mdp, .0001, .9));
	planner1->plan();
	Agent agent1(new _Agent(planner1));


	UCTPlanner planner2(new _FlatUCTPlanner(domain, mdp, 1, 1, false, true, 0));
	planner2->setRunLimit(-1);
	planner2->setTimeLimit(100);
	planner2->setConfidenceCoeff(1);
	Agent agent2(new _Agent(planner2));

	Environment env(new _MDPEnvironment(mdp, s));

	Experiment exp1(new _Experiment(env, agent1, 5));
	cout <<  exp1->runExperiment() << endl;

	Experiment exp2(new _Experiment(env, agent2, 5));
	cout <<  exp2->runExperiment() << endl;
}

void testState() {
	Domain domain(new _Domain());
	domain->addStateFactor(0, 1);
	domain->addStateFactor(0, 3);
	domain->addStateFactor(0, 3);
	domain->addStateFactor(0, 3);
	domain->addStateFactor(0, 3);
	domain->addStateFactor(0, 1);
	domain->addStateFactor(0, 1);
	domain->setRewardRange(-1, 1);

	const vector<Size>& v = domain->getStateIndexComponents();
	for (Size i=0; i<v.size(); i++)
		cout << v[i] << " ";
	cout << endl;

	State s(domain,0);
	cout << s << " - " << s.getIndex() << endl;
	s.setFactor(1,1);
	cout << s << " - " << s.getIndex() << endl;
	s.setFactor(2,1);
	cout << s << " - " << s.getIndex() << endl;
	s.setFactor(3,2);
	cout << s << " - " << s.getIndex() << endl;
	s.setFactor(4,2);
	cout << s << " - " << s.getIndex() << endl;
}

void testHash(Domain domain) {
	_HStateActionTable<int> hsat(domain, 10, 100);
	State s1(domain, 0);
	State s2(domain, 1);
	Action a1(domain, 0);
	Action a2(domain, 1);
//	hsat.setValue(s1, a1, 1);
//	hsat.setValue(s1, a2, 5);
	hsat.setValue(s2, a1, 3);
	hsat.setValue(s2, a2, 4);
	cout << hsat.getValue(s1, a1) << endl
		 << hsat.getValue(s1, a2) << endl
		 << hsat.getValue(s2, a1) << endl
		 << hsat.getValue(s2, a2) << endl;

	_HQTable qt(domain);
}

int main(int argc, char** argv) {
	try {
		srand(0);

		Domain domain(new _Domain());
		domain->addStateFactor(0, 299);
		domain->addActionFactor(0, 4);
		domain->setRewardRange(-1, 0);
		//MDP mdp = makeMDP(domain);
		MDP mdp = makeTerminatingMDP(domain);
		testState();
//		testVI(mdp, domain);
//		testPS(mdp, domain);
		testUCT(mdp, domain);
		//mdp->printXML(cout);
//		testState();
//		testVI(mdp, domain);
//		testHash(domain);
	}
	catch (Exception e) {
		cerr << e << endl;
	}
}
