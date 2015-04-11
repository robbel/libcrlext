/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef ENV_SYSADMIN_HPP_
#define ENV_SYSADMIN_HPP_

#include <iostream>
#include <crl/crl.hpp>
#include <crl/factor_learner.hpp> // for FactoredMDP

namespace crl {

/**
 * \brief The (multi-agent) SysAdmin environment with a variable number of computers arranged in a ring.
 * Follows the factored MDP model in Guestrin's thesis, sec 8.1
 */
class _Sysadmin : public _Environment {
protected:
  Domain _domain;
  State _current;
  // problem parameters
  /// \brief The number of computers in this ring
  Size _num_comps;
  Size _num_agents;
  /// \brief The reward function for the sysadmin problem
  virtual Reward getReward(const State& s) const;
public:
  /// \brief Create a SysAdmin problem from the supplied domain.
  _Sysadmin(Domain domain);
  virtual ~_Sysadmin() { }
  /// \brief Return domain associated with this SysAdmin instance
  virtual Domain getDomain() const {
    return _domain;
  }
  /// \brief Return the FactoredMDP representing this SysAdmin instance
  virtual FactoredMDP getFactoredMDP() const;

  //
  // Environment interface
  //
  /// \brief Return initial state
  virtual State begin() override;
  /// \brief Environment never reaches terminating state
  virtual bool isTerminated() override {
    return false;
  }
  /// \brief Apply the \a Action and return the resulting \a Observation
  virtual Observation getObservation(const Action& a) override;
};
typedef boost::shared_ptr<_Sysadmin> Sysadmin;

/// \brief Build a sysadmin problem with the specified number of computers in the ring
Sysadmin buildSysadmin(Size num_comps);

}

#endif /*ENV_SYSADMIN_HPP_*/
