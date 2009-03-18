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

#ifndef FACTOR_LEARNER_HPP_
#define FACTOR_LEARNER_HPP_

#include <boost/shared_ptr.hpp>
#include "crl/common.hpp"
#include "crl/crl.hpp"
#include "crl/flat_tables.hpp"

namespace crl {

typedef _StateActionTable<SizeVec> _SAFCountTable;
typedef boost::shared_ptr<_SAFCountTable> SAFCountTable;
typedef _StateActionTable<ProbabilityVec> _SAFProbTable;
typedef boost::shared_ptr<_SAFProbTable> SAFProbTable;

class _FactorLearner : public _Learner {
protected:
	Domain _domain;
	Domain _subdomain;
	Size _target;
	FactorRange _target_range;
	SizeVec _delayed_dep;
	SizeVec _concurrent_dep;
	SizeVec _action_dep;

	SACountTable _sa_count;
	SAFCountTable _sa_f_count;

	SAFProbTable _prob_table;

	State mapState(const State& s, const State& n);
	Action mapAction(const Action& a);
public:
	_FactorLearner(const Domain& domain, Size target);
	virtual ~_FactorLearner() { }
	virtual void addDelayedDependency(Size index);
	virtual void addConcurrentDependency(Size index);
	virtual void addActionDependency(Size index);
	virtual void pack();

	virtual bool observe(const State& s, const Action& a, const Observation& o);

	virtual StateDistribution augmentDistribution(StateDistribution sd, const State& s, const Action& a);
};
typedef boost::shared_ptr<_FactorLearner> FactorLearner;

class _FactorMDPLearner : public _MDPLearner {
protected:
	Domain _domain;
	std::vector<FactorLearner> _factor_learners;
public:
	_FactorMDPLearner(const Domain& domain);
	virtual ~_FactorMDPLearner() { }

	void addFactorLearner(FactorLearner& factor_learner);

	virtual StateIterator S();
	virtual StateIterator predecessors(const State& s);
	virtual ActionIterator A();
	virtual ActionIterator A(const State& s);
	virtual StateDistribution T(const State& s, const Action& a);
	virtual Reward R(const State& s, const Action& a);

	virtual bool observe(const State& s, const Action& a, const Observation& o);

};
typedef boost::shared_ptr<_FactorMDPLearner> FactorMDPLearner;

}

#endif /*FACTOR_LEARNER_HPP_*/
