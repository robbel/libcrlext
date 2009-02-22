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

#include <cpputil.hpp>

#include "crl/dpmem.hpp"

using namespace crl;
using namespace cpputil;

Size _DPMem::draw() {
	Probability u = randDouble();
	double total_mass = _total+_alpha;
	double r = u*total_mass;
	double c = 0;
	for (Size i=0; i<_counts.size(); i++) {
		c += _counts[i];
		if (c < r) {
			_counts[i]++;
			_total++;
			return i;
		}
	}
	Size value = _gen->next();
	return draw(value);
}

Size _DPMem::draw(Size value) {
	if (_counts.size() <= value) {
		_counts.resize(value, 0);
	}
	_counts[value]++;
	_total++;
	return value;
}

Size _DPMem::peekUnseen() {
	return _gen->peek();
}

void _DPMem::undraw(Size value) {
	_counts[value]--;
	_total--;
}

Size _DPMem::count(Size value) {
	if (_counts.size() > value)
		return _counts[value];
	return 0;
}

Probability _DPMem::P(Size value) {
	if (value >= _counts.size())
		return _alpha/_total;
	return _counts[value]/_total;
}

