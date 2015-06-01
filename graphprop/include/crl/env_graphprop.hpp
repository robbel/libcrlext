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
#include "crl/bigindex.hpp"

namespace crl {

namespace graphprop {

/**
 * \brief (Square) tabular storage implemented as flat, linear array
 * \todo Implement as sparse matrix (e.g., for adjacency map in sparse graph)
 */
template<class T>
class _FSizeSizeTable {
protected:
	Domain _domain;
	Size _num_states;
	std::vector<T> _ss_values;
public:
	_FSizeSizeTable(const Domain& domain)
	: _domain(domain), _num_states(domain->getNumStateFactors()), _ss_values(_num_states * _num_states) {
	}
	_FSizeSizeTable(const Domain& domain, T initial)
	: _domain(domain), _num_states(domain->getNumStateFactors()), _ss_values(_num_states * _num_states, initial) {
	}
	/// \brief copy ctor
	_FSizeSizeTable(const _FSizeSizeTable& rhs) = default;
	/// \brief move ctor
	_FSizeSizeTable(_FSizeSizeTable&&) = default;
	/// \brief dtor
	virtual ~_FSizeSizeTable() { }

	virtual void clear() {
	  _ss_values = std::vector<T>(_num_states * _num_states);
	}

	virtual T& getValue(const Size& si, const Size& sj) {
	  return _ss_values[si*_num_states+sj];
	}
	virtual void setValue(const Size& si, const Size& sj, T t) {
	  _ss_values[si*_num_states+sj] = t;
	}
	/**
	 * \brief Returns all values in this table
	 * (s,s) pairs are stored in row-major order, i.e., (s0,s0),(s0,s1),...,(s1,s0),(s1,s1), etc.
	 * \note Optimization when all elements have to be accessed in series
	 */
	virtual std::vector<T>& values() {
	  return _ss_values;
	}
	/// \brief Return iterator to row `i' of this table
	virtual typename std::vector<T>::const_iterator getRow(Size r) {
	  return _ss_values.begin()+r*_num_states;
	}
	/// \brief Transpose this table
	std::vector<T> transpose() {
	  std::vector<T> ret_vec(_ss_values.size());
	  #pragma omp parallel for
	  for(Size r = 0; r<_num_states; r++) {
	      auto rit = getRow(r);
	      Size c = 0;
	      std::for_each(rit,rit+_num_states,[&] (T n){
		ret_vec[c*_num_states+r] = n;
		c++;
	      });
	  }
	  return ret_vec;
	}
};
template<class T>
using FSizeSizeTable = boost::shared_ptr<_FSizeSizeTable<T>>;
typedef _FSizeSizeTable<int> _AdjacencyMap;
typedef FSizeSizeTable<int>   AdjacencyMap;

/**
 * \brief The GraphProp environment.
 * This implements a (controllable) version of disease propagation over a graph problem described in:
 * "Control of Epidemics on Graphs" by Christopher Ho, Mykel J. Kochenderfer,
 * Vineet Mehta, and Rajmonda S. Caceres, 2015 (in publication)
 * \note Implemented as an infinite-horizon problem.
 */
class _GraphProp : public _BigEnvironment {
protected:
  Domain _domain;
  FactoredMDP _fmdp;
  /// \brief The (undirected or directed) graph underlying this propagation model (no self-loops)
  AdjacencyMap _adj_map;
  /// \brief An optimized accessor for parents (including self) in the graph
  /// \note the mapped element (SizeVec) is in order of increasing indices
  std::unordered_map<Size,SizeVec> _scope_map;
  /// \brief Current joint state
  BigState _current;
  /// \brief Sorted location (i.e., node) vector for each agent (i.e., controllable node) in the graph
  /// \note Assumed disjoint from _target_locs
  std::vector<Size> _agent_locs;
  /// \brief Array of size `_num_nodes' for quick checks whether agent is present at a node
  std::vector<bool> _agent_active;
  /// \brief Sorted location (i.e., node) vector for each target in the graph
  /// \note Assumed disjoint from _agent_locs
  std::vector<Size> _target_locs;
  /// \brief Array of size `_num_nodes' for quick checks whether target is present at a node
  std::vector<bool> _target_active;
  // problem parameters
  Size _num_nodes;   ///< Nodes in graph
  Size _num_agents;  ///< Controlled nodes in graph
  Size _num_targets; ///< Target nodes in graph
  double _beta0, _del0, _lambda1, _lambda2, _lambda3, _q0;
  /// \brief Graph diffusion model computed from _beta0 (no self-infusion)
  /// Stored in transposed fashion: \a getRow(i) returns \a incoming rates to node `i'
  /// \see setParameters
  _FSizeSizeTable<double> _beta_t;
  /// \brief Per-node recovery model computed from _del0
  /// \see setParameters
  std::vector<double> _del;

  /// \brief The reward function for the GraphProp problem defined over current state and action
  virtual Reward getReward(const BigState& s, const Action &a) const;
  /// \brief Fill in transition and reward function for a single plate (i.e., node `i') in the graph
  void buildPlate(Size i, DBNFactor& fai, LRF& lrf);
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
  virtual void setParameters(double beta0, double del0, double lambda1, double lambda2, double lambda3, double q0);

  //
  // Environment interface
  //
  /// \brief Return initial state
  virtual BigState begin() override;
  /// \brief True iff environment has reached a terminating state
  virtual bool isTerminated() override {
    return false;
  }
  /// \brief Apply the \a Action and return the resulting \a Observation
  virtual BigObservation getObservation(const Action& a) override;
};
typedef boost::shared_ptr<_GraphProp> GraphProp;

} // namespace graphprop

/// \brief read in GraphProp details from xml config and graph (matrix-form) file
graphprop::GraphProp readGraphProp(std::istream& cfg, std::istream& graph);

} // namespace crl

#endif /*ENV_GRAPHPROP_HPP_*/
