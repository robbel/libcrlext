/*
    Copyright 2008, 2009 Rutgers University
    Copyright 2008, 2009 John Asmuth

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

#ifndef RAM_HPP_
#define RAM_HPP_

#include <map>
#include <boost/shared_ptr.hpp>
#include "cpputil.hpp"
#include "crl/crl.hpp"
#include "crl/flat_tables.hpp"
#include "crl/hash_tables.hpp"

namespace crl {

class _Outcome {
	Size _index;
public:
	virtual ~_Outcome() {}
	virtual StateDistribution T(const State& s) = 0;
	virtual Probability P(const State& s) = 0;
	virtual void setIndex(Size index) {
		_index = index;
	}
	virtual Size getIndex() {
		return _index;
	}
};
typedef boost::shared_ptr<_Outcome> Outcome;
typedef boost::shared_ptr<cpputil::Iterator<Outcome> > OutcomeIterator;
typedef boost::shared_ptr<std::set<Outcome> > OutcomeSet;


class _OutcomeDistribution : public _Distribution<Outcome> {
protected:
	OutcomeSet _outcomes;
	std::vector<Probability> _probabilities;
public:
	_OutcomeDistribution();
	virtual ~_OutcomeDistribution() {}
	virtual OutcomeIterator iterator();
	virtual Probability P(const Outcome& o);
	virtual void setP(const Outcome& o, Probability p);
	virtual void clear();
};
typedef boost::shared_ptr<_OutcomeDistribution> OutcomeDistribution;

typedef boost::shared_ptr<_ActionTable<OutcomeDistribution> > ActionOutcomeDistributionTable;

class _Cluster {
protected:
	Domain _domain;
	OutcomeSet _outcomes;
	ActionOutcomeDistributionTable _action_outcome_distributions;
public:
	_Cluster(Domain domain);
	virtual ~_Cluster() {}
	virtual StateDistribution T(const State& s, const Action& a);
	virtual Probability P(const State& s, const Action& a, const State& n);
};
typedef boost::shared_ptr<_Cluster> Cluster;

class _ClusterMapping : public _StateTable<Cluster> {
public:
	virtual ~_ClusterMapping() { }
	virtual Cluster getCluster(const State& s) {
		return getValue(s);
	}
	virtual void setCluster(const State& s, const Cluster& c) {
		setValue(s, c);
	}
};
typedef boost::shared_ptr<_ClusterMapping> ClusterMapping;

class _FClusterMapping : public _ClusterMapping, _FStateTable<Cluster> {
public:
	_FClusterMapping(const Domain& domain)
	: _FStateTable<Cluster>(domain) { }
	virtual ~_FClusterMapping() { }
};

class _HClusterMapping : public _ClusterMapping, _HStateTable<Cluster> {
public:
	_HClusterMapping()
	: _HStateTable<Cluster>() { }
	virtual ~_HClusterMapping() { }
};

}

#endif /* RAM_HPP_ */
