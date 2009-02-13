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

#include <crl/crl.hpp>
#include <crl/hdomain.hpp>
#include <crl/fdomain.hpp>

namespace crl {

class _Outcome {
public:
	virtual bool match(const State& s, const State& sp) = 0;
	virtual ~_Outcome() { }
};
typedef boost::shared_ptr<_Outcome> Outcome;

class _StepOutcome : public _Outcome {
protected:
	const std::vector<int> _deltas;
public:
	_StepOutcome(const std::vector<int>& deltas);
	virtual ~_StepOutcome() { }
	virtual bool match(const State& s, const State& sp);
};
typedef boost::shared_ptr<_StepOutcome> StepOutcome;

class _FixedOutcome : public _Outcome {
protected:
	const State _s;
public:
	_FixedOutcome(const State& s);
	virtual ~_FixedOutcome() { }
	virtual bool match(const State& s, const State& sp);
};
typedef boost::shared_ptr<_FixedOutcome> FixedOutcome;

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
	virtual Probability outcomesGivenDistribution(const State& s, std::vector<_FActionTable<Probability> > dist);
	virtual std::vector<_FActionTable<Probability> > clusterDistribution(StateIterator sitr);
};

};

#endif /* OUTCOMES_HPP_ */
