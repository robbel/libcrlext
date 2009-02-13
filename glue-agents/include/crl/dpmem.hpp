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

#ifndef DPMEM_HPP_
#define DPMEM_HPP_

#include <vector>
#include <crl/common.hpp>

namespace crl {

class Generator {
public:
	virtual Size next() = 0;
	virtual void recycle(Size value) = 0;
	virtual ~Generator() { }
};

class IncrementGenerator : public Generator {
protected:
	Size next_value;
	std::vector<Size> recycling_bin;
public:
	IncrementGenerator() {
		next_value = 0;
	}
	virtual Size next() {
		if (recycling_bin.size() > 0) {
			Size value = recycling_bin[recycling_bin.size()-1];
			recycling_bin.pop_back();
			return value;
		}
		return next_value++;
	}
	virtual void recycle(Size value) {
		recycling_bin.push_back(value);
	}
};

class DPMem {
protected:
	crl::Probability _alpha;
	Generator* _gen;
	std::vector<Size> _counts;
	size_t _total;
	DPMem(crl::Probability alpha, Generator* gen)
	: _alpha(alpha), _gen(gen) {

	}
public:
	/**
	 * draw a value according to the DP
	 */
	Size draw();
	/**
	 * force a value to be drawn
	 */
	Size draw(Size value);
	/**
	 * decrement the count of one value
	 */
	void undraw(Size value);
	/**
	 * get the number of times this value was drawn
	 */
	Size count(Size value);
	/**
	 * Get the posterior of a possible sample
	 */
	Probability P(Size value);

	virtual ~DPMem() { }
};

class IncrementDPMem : public DPMem {
public:
	IncrementDPMem(crl::Probability alpha)
	: DPMem(alpha, new IncrementGenerator()){
	}
	~IncrementDPMem() {
		delete _gen;
	}
};

};

#endif /* DPMEM_HPP_ */
