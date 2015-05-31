/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef BIGINDEX_HPP_
#define BIGINDEX_HPP_

#include <iostream>
#include <crl/common.hpp>
#include <crl/crl.hpp>

namespace crl {

/**
 * \brief A big state in a \a Domain (i.e., as per structure definition there)
 * Unlike a \a State, the \a BigState does not use numerical indexing to support domains with
 * more than 2^64 states.
 */
class BigState : public State {
private:
        // trick to change visibility to private
        using RLType::_ranges;
        using RLType::_components;
        using RLType::_index;
        using RLType::setIndex;
        using RLType::getIndex;
        using RLType::operator Size;
protected:
        /// \brief The vector of state factor values
        FactorVec _values;
public:
	BigState()
	: State() { }
	explicit BigState(const Domain& domain)
	: State(), _values(domain->getNumStateFactors(), 0) { }
	BigState(const Domain& domain, const FactorVec& index)
	: State(), _values(index) { }

	Size size() const {
	  return _values.size();
	}

	///
	/// \brief set the value associated with the \a Factor at \a index
	///
	void setFactor(Size index, Factor data) {
	  _values[index] = data;
	}
	///
	/// \brief get the value associated with the \a Factor at \a index.
	///
	const Factor getFactor(Size index) const {
	  return _values[index];
	}

	void print(std::ostream& os) const;
	bool operator==(const BigState& r) const {
		return _values == r._values;
	}
};

/**
 * A pair of BigState,Reward is referred to as a BigObservation
 */
class _BigObservation : public _Observation {
private:
        // trick to change visibility to private
        using _Observation::getState;
        using _Observation::_s;
protected:
        BigState _big_s;
public:
	_BigObservation(const BigState& s, Reward r)
	: _Observation(State(), r), _big_s(s) { }

	BigState getState() const {return _big_s;}
};
typedef boost::shared_ptr<_BigObservation> BigObservation;

/**
 * Interface for a big RL environment that starts in one \a BigState and generates successor \a BigObservation
 * given an agent action.
 * \note This is for large environments with more than 2^64 states (e.g., \a GraphProp)
 */
class _BigEnvironment {
public:
	virtual ~_BigEnvironment() { }

	virtual BigState begin() = 0;
	virtual bool isTerminated() = 0;
	virtual BigObservation getObservation(const Action& a) = 0;
};
typedef boost::shared_ptr<_BigEnvironment> BigEnvironment;

} // namespace crl

#endif /*BIGINDEX_HPP_*/
