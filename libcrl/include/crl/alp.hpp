/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef ALP_HPP_
#define ALP_HPP_

#include <iostream>
#include "crl/dbn.hpp"
#include "crl/factor_learner.hpp"

#if 0 // example invokation:
Action testFALP(FactoredMDP fmdp, Domain domain) {
	long start_time = time_in_milli();
	cout << "FALP" << endl;
	ALPPlanner planner(new _ALPPlanner(fmdp, .9));
	planner->plan();
	long end_time = time_in_milli();
	QTable qtable = planner->getQTable();
	State s(domain, 0);
	cout << " V(" << s << ") = " << qtable->getV(s) << endl;
	Action a = planner->getAction(s);
	cout << " best action = " << a << endl;
	cout << "FVI finished " << count << " iterations in " << end_time - start_time << endl;
	return a;
}
#endif

namespace crl {

/**
 * \brief A factored value function
 * Consists of locally-scoped basis functions along with their weights
 */
class _FactoredValueFunction {
protected:
  /// \brief The (global) domain of this value function
  const Domain _domain;
public:
  _FactoredValueFunction() { }

  /// \brief Add a basis function (with local state and action scope) to this V-fn
  void addBasisFunction(const DiscreteFunction<Reward>& h);

  /// \brief Return value of (global) \a State s
  virtual Reward eval(const State& s) const;
};
typedef boost::shared_ptr<_FactoredValueFunction> FactoredValueFunction;

/**
 * A planner that uses an approximate linear program (ALP) along with a factored value function
 * \see Guestrin, Koller, Parr, and Venkatarman, 2003
 */
class _ALPPlanner : public _Planner {
protected:
  Domain _domain;
  /// \brief The factored dynamics and reward models
  FactoredMDP _fmdp;
  /// \brief The computed factored value function
  // todo...
  /// \brief Discount factor
  float _gamma;
  /// \brief Whether all basis functions have been cached
  bool _cached;
public:
  _ALPPlanner(const FactoredMDP& fmdp, float gamma)
  : _domain(fmdp->getDomain()), _fmdp(fmdp), _gamma(gamma), _cached(false) { }

  virtual Action getAction(const State& s) override {

    // todo...
    // Action a = _qfunction->getBestAction(s);

    return Action();
  }

  ///
  /// \brief run the FactoredALP algorithm
  ///
  int plan();

//  QFunction getQFunction() {
//     return _qfunction;
//  }

};
typedef boost::shared_ptr<_ALPPlanner> ALPPlanner;

} // namespace crl

#endif
