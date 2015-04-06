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

#include <set>
#include <cpputil.hpp>
#include "crl/ram.hpp"
#include "crl/flat_tables.hpp"

using namespace std;
using namespace cpputil;
using namespace crl;

_OutcomeDistribution::_OutcomeDistribution()
: _outcomes(new set<Outcome>()) {

}

OutcomeIterator _OutcomeDistribution::iterator() {
	return OutcomeIterator(new cpputil::SharedContainerIterator<Outcome, set<Outcome> >(_outcomes));
}

Probability _OutcomeDistribution::P(const Outcome& o) {
	return _probabilities[o->getIndex()];
}

void _OutcomeDistribution::setP(const Outcome& o, Probability p) {
	_outcomes->insert(o);
	if (o->getIndex() >= _probabilities.size())
		_probabilities.resize(o->getIndex()+1, 0);
	_probabilities[o->getIndex()] = p;
}

void _OutcomeDistribution::clear() {
	_outcomes->clear();
	_probabilities.clear();
}


_Cluster::_Cluster(Domain domain)
: _domain(domain) {

}

StateDistribution _Cluster::T(const State& s, const Action& a) {
	FStateDistribution sd(new _FStateDistribution(_domain));
/*
	OutcomeDistribution od = _action_outcome_distributions->getValue(a);
	OutcomeIterator oitr = od->iterator();
	while (oitr->hasNext()) {
		const Outcome& o = oitr->next();
		StateDistribution osd = o->T(s);
		StateIterator sitr = osd->iterator();
		while (sitr->hasNext()) {
			const State& s = sitr->next();
			Probability p = od->P(o)*osd->P(s);
			sd->setP(s, p+sd->P(s));
		}
	}
*/
	return sd;
}

Probability _Cluster::P(const State& s, const Action& a, const State& n) {
	Probability p = 0;
/*
	OutcomeDistribution od = _action_outcome_distributions->getValue(a);
	OutcomeIterator oitr = od->iterator();
	while (oitr->hasNext()) {
		const Outcome& o = oitr->next();
		StateDistribution osd = o->T(s);
		p += od->P(o)*osd->P(n);
	}
*/
	return p;
}

