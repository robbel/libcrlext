/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth
    Copyright 2015 Philipp Robbel

    This file is part of CRL:RL-Glue.

    CRL:RL-Glue is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CRL:RL-Glue is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CRL:RL-Glue.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef GLUE_AGENT_HPP_
#define GLUE_AGENT_HPP_

#include <crl/crl.hpp>
#include <crl/bigindex.hpp>
#include <rlglue/utils/C/TaskSpec_Parser.h>

namespace crl {

///
/// \brief An interface to the domain and state for the desired \a crl::Agent
/// This can be used to convert between, e.g., a continuous rl-glue domain to a discretized crl::Domain.
/// \see _AcrobotMapper
///
template<class S>
class _StateMapperBase {
public:
	using state_t = S;

	virtual ~_StateMapperBase() { }
	virtual Domain getDomain(Domain old_domain, taskspec_t* task_spec) {
		return old_domain;
	}
	virtual S getState(Domain domain, const observation_t* obs) {
		S s(domain);
		for (Size i=0; i<domain->getNumStateFactors(); i++)
			s.setFactor(i, obs->intArray[i]);
		return s;
	}
};
template<class S>
using StateMapperBase = boost::shared_ptr<_StateMapperBase<S>>;
typedef _StateMapperBase<State> _StateMapper;
typedef boost::shared_ptr<_StateMapper> StateMapper;
typedef _StateMapperBase<BigState> _BigStateMapper;
typedef boost::shared_ptr<_BigStateMapper> BigStateMapper;

///
/// \brief Construct and obtain \a StateMapper for the desired \a crl::Agent
/// \note Does not currently support custom state mappers for big domains (> 2^64 states)
/// \see agent_init()
///
StateMapper getStateMapper();

///
/// \brief getCRLAgent Construct and obtain desired \a crl::Agent
/// \see agent_init()
///
Agent getCRLAgent(Domain domain);

}

#endif /*GLUE_HPP_*/
