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
 
#ifndef ENVIRONMENT_HPP_
#define ENVIRONMENT_HPP_

#include <boost/shared_ptr.hpp>
#include "crl/crl.hpp"

namespace crl {

/**
 * An environment that samples from a given MDP and start state.
 */
class _MDPEnvironment : public _Environment {
protected:
	const MDP _mdp;
	const State _start_state;
	State _current_state;
public:
	_MDPEnvironment(const MDP& mdp, const State& start_state);
	virtual ~_MDPEnvironment() { }
	
	virtual State begin() override;
	virtual bool isTerminated() override;
	virtual Observation getObservation(const Action& a) override;
};
typedef boost::shared_ptr<_MDPEnvironment> MDPEnvironment;
	
}

#endif /*ENVIRONMENT_HPP_*/
