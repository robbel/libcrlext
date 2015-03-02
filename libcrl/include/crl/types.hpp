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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <set>
#include <boost/cstdint.hpp>

namespace crl {

typedef float Reward;
typedef double Probability;
typedef short Factor;  ///< The value type of a (discrete) Factor in the graph
typedef uint64_t Size;
typedef long Index;

typedef cpputil::Range<Reward> RewardRange;
typedef cpputil::Range<Factor> FactorRange;

typedef std::vector<Factor> FactorVec;
typedef std::vector<FactorRange> RangeVec;
typedef std::vector<Probability> ProbabilityVec;
typedef std::vector<Size> SizeVec;

class State;
class Action;

typedef std::set<State> _StateSet;
typedef boost::shared_ptr<_StateSet> StateSet;

typedef std::set<Action> _ActionSet;
typedef boost::shared_ptr<_ActionSet> ActionSet;

typedef cpputil::EmptyIterator<State> _EmptyStateIterator;
typedef cpputil::EmptyIterator<Action> _EmptyActionIterator;

typedef boost::shared_ptr<_EmptyStateIterator> EmptyStateIterator;
typedef boost::shared_ptr<_EmptyActionIterator> EmptyActionIterator;

typedef cpputil::Iterator<State> _StateIterator;
typedef cpputil::Iterator<Action> _ActionIterator;
typedef cpputil::ContainerIterator<State,_StateSet> _StateSetIterator;
typedef cpputil::ContainerIterator<Action,_ActionSet> _ActionSetIterator;
typedef cpputil::SharedContainerIterator<State,_StateSet> _SharedStateSetIterator;
typedef cpputil::SharedContainerIterator<Action,_ActionSet> _SharedActionSetIterator;

typedef boost::shared_ptr<_StateIterator> StateIterator;
typedef boost::shared_ptr<_ActionIterator> ActionIterator;
typedef boost::shared_ptr<_StateSetIterator> StateSetIterator;
typedef boost::shared_ptr<_ActionSetIterator> ActionSetIterator;
typedef boost::shared_ptr<_SharedStateSetIterator> SharedStateSetIterator;
typedef boost::shared_ptr<_SharedActionSetIterator> SharedActionSetIterator;

}

#endif /*TYPES_HPP_*/
