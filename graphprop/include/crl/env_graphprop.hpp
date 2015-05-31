/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef ENV_GRAPHPROP_HPP_
#define ENV_GRAPHPROP_HPP_

#include <iostream>
#include <crl/crl.hpp>
#include <crl/factor_learner.hpp> // for FactoredMDP

namespace crl {

namespace graphprop {

/**
 * \brief (Square) tabular storage implemented as flat, linear array
 * \todo Implement as sparse matrix (e.g., for adjacency map in sparse graph)
 */
template<class T>
class _FStateStateTable {
protected:
	Domain _domain;
	Size _num_states;
	std::vector<T> _ss_values;
public:
	_FStateStateTable(const Domain& domain)
	: _domain(domain), _num_states(domain->getNumStates()), _ss_values(_num_states * _num_states) {
	}
	_FStateStateTable(const Domain& domain, T initial)
	: _domain(domain), _num_states(domain->getNumStates()), _ss_values(_num_states * _num_states, initial) {
	}
	/// \brief copy ctor
	_FStateStateTable(const _FStateStateTable& rhs) = default;
	/// \brief move ctor
	_FStateStateTable(_FStateStateTable&&) = default;
	/// \brief dtor
	virtual ~_FStateStateTable() { }

	virtual void clear() {
	  _ss_values = std::vector<T>(_num_states * _num_states);
	}

	virtual T& getValue(const State& si, const State& sj) {
	  return _ss_values[si.getIndex()*_num_states+sj.getIndex()];
	}
	virtual void setValue(const State& si, const State& sj, T t) {
	  _ss_values[si.getIndex()*_num_states+sj.getIndex()] = t;
	}
	/**
	 * \brief Returns all values in this table
	 * (s,s) pairs are stored in row-major order, i.e., (s0,s0),(s0,s1),...,(s1,s0),(s1,s1), etc.
	 * \note Optimization when all elements have to be accessed in series
	 */
	virtual std::vector<T>& values() {
	  return _ss_values;
	}
};
template<class T>
using FStateStateTable = boost::shared_ptr<_FStateStateTable<T>>;
typedef _FStateStateTable<int> _AdjacencyMap;
typedef FStateStateTable<int>   AdjacencyMap;

/**
 * \brief The GraphProp environment.
 * This implements a (controllable) version of disease propagation over a graph problem described in:
 * "Control of Epidemics on Graphs" by Christopher Ho, Mykel J. Kochenderfer,
 * Vineet Mehta, and Rajmonda S. Caceres, 2015 (in publication)
 * \note Implemented as an infinite-horizon problem.
 */
class _GraphProp : public _Environment {
protected:
  Domain _domain;
  FactoredMDP _fmdp;
  /// \brief The graph underlying this propagation model
  AdjacencyMap _adj_map;
  /// \brief Current joint state
  State _current;
  // problem parameters
  Size _num_nodes;
  Size _num_agents;
  Size _num_targets;
  double _beta0, _del0, _lambda1, _lambda2, _lambda3, _q0;
  /// \brief Sorted location (i.e., node) vector for each agent in the graph
  /// \note Assumed comparably small to \a _num_nodes.
  std::vector<Size> _agent_locs;
  /// \brief Sorted location (i.e., node) vector for each target in the graph
  /// \note Assumed comparably small to \a _num_nodes.
  std::vector<Size> _target_locs;

  /// \brief The reward function for the GraphProp problem
  virtual Reward getReward(const State& n) const;
public:
  /// \brief Create a GraphProp environment from the supplied domain
  _GraphProp(Domain domain, AdjacencyMap adj_map);
  virtual ~_GraphProp() { }
  /// \brief Return domain associated with this GraphProp
  virtual Domain getDomain() const {
    return _domain;
  }
  /// \brief Build the factored MDP associated with this GraphProp problem
  virtual void buildFactoredMDP();
  /// \brief Return the FactoredMDP representing this GraphProp instance
  virtual FactoredMDP getFactoredMDP() const {
      return _fmdp;
  }
  /// \brief Parse given location string (e.g., from xml file) into an agent-to-location assignment
  /// \note If the empty string is given, agent locations are randomized
  virtual void setAgentLocs(Size num_agents, std::string locs = "");
  virtual void setTargetLocs(Size num_targets, std::string locs = "");
  /// \brief Set problem parameters
  virtual void setParameters(double beta0, double del0, double lambda1, double lambda2, double lambda3, double q0)
  {
    _beta0 = beta0;
    _del0 = del0;
    _lambda1 = lambda1;
    _lambda2 = lambda2;
    _lambda3 = lambda3;
    _q0 = q0;
  }

  //
  // Environment interface
  //
  /// \brief Return initial state
  virtual State begin() override;
  /// \brief True iff environment has reached a terminating state
  virtual bool isTerminated() override {
    return false;
  }
  /// \brief Apply the \a Action and return the resulting \a Observation
  virtual Observation getObservation(const Action& a) override;
};
typedef boost::shared_ptr<_GraphProp> GraphProp;

} // namespace graphprop

/// \brief read in GraphProp details from xml config and graph (matrix-form) file
graphprop::GraphProp readGraphProp(std::istream& cfg, std::istream& graph);

} // namespace crl

#endif /*ENV_GRAPHPROP_HPP_*/
