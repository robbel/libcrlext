/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef ENV_FFG_HPP_
#define ENV_FFG_HPP_

/*
 create a FFG domain template
 or: generalized domain
 or: generalized environment (rl-glue lingo)
 */

namespace crl {

/**
 * \brief The FireFightingGraph environment.
 * This corresponds to a fully-observable version of the FireFightingGraph problem described in:
 * "Value-Based Planning for Teams of Agents in Stochastic Partially Observable Environments", Frans A. Oliehoek, 2010
 * \note This is a template that can be instantiated for different houses, firelevels, etc.
 */
class _FireFightingGraph : public _Environment {
protected:
  Domain _domain;
  State _current;
public:
  _FireFightingGraph(Domain domain)
  : _domain(std::move(domain)) { }
  virtual ~_FireFightingGraph() { }

  //
  // Environment interface
  //

  /// \brief Return initial state
  virtual State begin() override;
  /// \brief True iff environment has reached a terminating state
  virtual bool isTerminated() override;
  /// \brief Apply the \a Action and return the resulting \a Observation
  virtual Observation getObservation(const Action& a) override;

};

}

#endif /*ENV_FFG_HPP_*/
