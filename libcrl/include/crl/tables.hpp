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

#ifndef TABLES_HPP_
#define TABLES_HPP_

#include <vector>
#include <limits>
#include <boost/shared_ptr.hpp>
#include <cpputil.hpp>
#include "crl/crl.hpp"

namespace crl {

/**
 * An interface for a table that uses states as keys
 */
template <class T>
class _StateTable {
public:
	virtual ~_StateTable() { }

	virtual T& getValue(const State& s) = 0;
	virtual void setValue(const State& s, T t) = 0;
	virtual StateIterator iterator() = 0;
};
typedef boost::shared_ptr<_StateTable<Index> > SCountTable;

/**
 * An interface for a table that uses actions as keys
 */
template <class T>
class _ActionTable {
public:
	virtual ~_ActionTable() { }

	virtual T& getValue(const Action& a) = 0;
	virtual void setValue(const Action& a, T t) = 0;
	virtual ActionIterator iterator() = 0;
};
typedef boost::shared_ptr<_ActionTable<Size> > ACountTable;

/**
 * An interface for a table that uses state/action pairs as keys
 */
template <class T>
class _StateActionTable {
public:
	virtual ~_StateActionTable() { }

	virtual T& getValue(const State& s, const Action& a) = 0;
	virtual void setValue(const State& s, const Action& a, T t) = 0;
};
typedef boost::shared_ptr<_StateActionTable<Index> > SACountTable;
typedef boost::shared_ptr<_StateActionTable<SCountTable> > SASCountTable;
typedef boost::shared_ptr<_StateActionTable<Reward> > SARTable;


/**
 * \brief An abstract interface for keeping track of times (state/action) pairs have been observed.
 */
class _Counter : public _Learner {
protected:
public:
	virtual ~_Counter() { }
	/**
	 * returns an iterator over all observed next states from s,a
	 */
	virtual StateIterator iterator(const State& s, const Action& a) = 0;
	virtual Size getCount(const State& s, const Action& a) = 0;
	virtual Size getCount(const State& s, const Action& a, const State& n) = 0;
	virtual bool observe(const State& s, const Action& a, const Observation& o) override = 0;
};
typedef boost::shared_ptr<_Counter> Counter;

}

#endif /* TABLES_HPP_ */
