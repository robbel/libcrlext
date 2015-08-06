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

namespace sysadmin {

/// \brief Computer status in the network
enum class Status : Factor {
    GOOD,
    FAULTY,
    DEAD
};
/// \brief The load variables
enum class Load : Factor {
    IDLE,
    LOADED,
    PROCESS_SUCCESS
};
/// \brief Admin actions
enum class Admin : Factor {
    NOTHING,
    REBOOT
};
/// \brief The type of architecture
enum class Topology {
    RING,
    STAR
};

/**
 * \brief The (multi-agent) SysAdmin environment with a variable number of computers arranged in a ring or star.
 * Follows the factored MDP model in Guestrin's thesis, secs 8.1 and 13.5.1
 */
class _Sysadmin : public _Environment {
protected:
  const Domain _domain;
  const Topology _network;
  FactoredMDP _fmdp;
  /// \brief Current joint state
  State _current;
  // problem parameters
  /// \brief The number of computers in this ring
  const Size _num_comps;
  const Size _num_agents;

  /// \brief Create a SysAdmin problem from the supplied domain and computer/agent numbers.
  _Sysadmin(Domain domain, Topology network, Size num_comps, Size num_agents);
  /// \brief The reward function for the sysadmin problem
  /// \note Agents do not incur action costs
  virtual Reward getReward(const State& s, const Action& a) const;
  /// \brief Build the factored MDP associated with this SysAdmin problem
  virtual void buildFactoredMDP();
  /// \brief Fill in transition and reward function for a single plate (i.e., computer `c') in the network
  virtual void buildPlate(Size c, DBNFactor& fas, DBNFactor& fal, LRF& lrf);
public:
  /// \brief Create a SysAdmin problem from the supplied domain.
  _Sysadmin(Domain domain, Topology network);
  virtual ~_Sysadmin() { }
  /// \brief Return domain associated with this SysAdmin instance
  virtual Domain getDomain() const {
    return _domain;
  }
  /// \brief Return the FactoredMDP representing this SysAdmin instance
  virtual FactoredMDP getFactoredMDP() const {
      return _fmdp;
  }

  //
  // Environment interface
  //
  /// \brief Return initial state
  virtual State begin() override;
  /// \brief Environment never reaches terminating state
  virtual bool isTerminated() override {
    return false;
  }
  /// \brief Apply the (joint) \a Action and return the resulting \a Observation
  virtual Observation getObservation(const Action& ja) override;
};
typedef boost::shared_ptr<_Sysadmin> Sysadmin;

/**
 * \brief The (simple, binary) SysAdmin environment with a variable number of computers arranged in a ring or star.
 * Follows the factored MDP model used in the IPPC planning competitions.
 * \see sysadmin_mdp.rddl
 */
class _SimpleSysadmin : public _Sysadmin {
protected:
  /// \brief The reward function for the simple sysadmin problem
  /// \note Agents do incur action costs
  virtual Reward getReward(const State& s, const Action& a) const override;
  /// \brief Build the factored MDP associated with this SysAdmin problem
  virtual void buildFactoredMDP() override;
  /// \brief Fill in transition and reward function for a single plate (i.e., computer `c') in the network
  virtual void buildPlate(Size c, DBNFactor& fas, DBNFactor& fal, LRF& lrf) override;
public:
  /// \brief The probability of a currently not running machine rebooting automatically
  static const double REBOOT_PROB;
  /// \brief The action penalty for rebooting a single computer
  static const double REBOOT_PENALTY;

  /// \brief Create a SysAdmin problem from the supplied domain.
  _SimpleSysadmin(Domain domain, Topology network);
  virtual ~_SimpleSysadmin() { }

  //
  // Environment interface
  //
  /// \brief Return initial state
  virtual State begin() override;
};

} // namespace sysadmin

/// \brief Build a sysadmin problem with the specified number of computers in the topology
/// \param arch The network topology, either "star" or "ring"
/// \param num_comps The number of computers in the network
sysadmin::Sysadmin buildSysadmin(std::string arch, Size num_comps);
/// \brief Build a simple sysadmin problem with the specified number of computers in the topology
/// A `simple' sysadmin corresponds to the binary version used in the IPPC competitions
/// \param arch The network topology, either "star" or "ring"
/// \param num_comps The number of computers in the network
sysadmin::Sysadmin buildSimpleSysadmin(std::string arch, Size num_comps);

} // namespace crl

#endif /*ENV_SYSADMIN_HPP_*/
