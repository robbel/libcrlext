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
 * \note There is still the assumption that all subdomains (e.g., function domains) fit in a regular \a State.
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
        using RLType::operator<;
        using RLType::operator==;
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

	virtual Size size() const override {
	  return _values.size();
	}

	///
	/// \brief set the value associated with the \a Factor at \a index
	///
	virtual void setFactor(Size index, Factor data) override {
	  _values[index] = data;
	}
	///
	/// \brief get the value associated with the \a Factor at \a index.
	///
	virtual const Factor getFactor(Size index) const override {
	  return _values[index];
	}

	virtual void print(std::ostream& os) const override;
	bool operator==(const BigState& r) const {
		return _values == r._values;
	}
	virtual operator bool() const override {
		return !_values.empty();
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

	virtual const State& getState() const override {return _big_s;}
};
typedef boost::shared_ptr<_BigObservation> BigObservation;

/**
 * Interface for a big RL environment that starts in one \a BigState and generates successor \a BigObservation
 * given an agent action.
 * \note This is for large environments with more than 2^64 states (e.g., \a GraphProp)
 * \note A _BigEnvironment is not an _Environment
 */
class _BigEnvironment {
public:
        typedef BigState state_t;
        typedef BigObservation observation_t;

	virtual ~_BigEnvironment() { }

	virtual BigState begin() = 0;
	virtual bool isTerminated() = 0;
	virtual BigObservation getObservation(const Action& a) = 0;
};
typedef boost::shared_ptr<_BigEnvironment> BigEnvironment;

/**
 * An experiment interface for a \a BigEnvironment.
 */
typedef _ExperimentBase<_BigEnvironment> _BigExperiment;
typedef boost::shared_ptr<_BigExperiment> BigExperiment;

} // namespace crl

#endif /*BIGINDEX_HPP_*/
