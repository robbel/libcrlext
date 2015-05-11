/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef SPUDD_HPP_
#define SPUDD_HPP_

#include "crl/crl.hpp"

#include "MDP.h" // main SPUDD header

/// \brief Overrides SPUDD's version to make sure memory is dealt with correctly
/// Ensures that base class dtor goes through correctly
class SPUDDMDP : public ::MDP {
public:
    /// \brief Return action and value of optimal policy in given state
    std::tuple<int,double> consultOptimalPolicyAV(DdNode *pol, DdNode *val, int *varvals);
    /// \brief patched dtor
    ~SPUDDMDP();
};

namespace crl {

/**
 * A wrapper for a pre-computed SPUDD policy
 */
class _SpuddPolicy : public _Policy {
protected:
    Domain _domain;
    /// \brief The SPUDD mdp/policy representation
    /// \warning The dtor for ::MDP in the SPUDD 3.6.2 library is currently broken and will segfault
    /// FIXME work-around
    boost::shared_ptr<SPUDDMDP>  _mdp;
    DdNode* _act;
    DdNode* _val;
public:
    /// \brief Load a SPUDD policy from a dual ADD file
    _SpuddPolicy(const Domain& domain, const char* filename);
    /// \brief dtor
    /// \warning The dtor for ::MDP in the SPUDD library is currently broken and will segfault
    /// FIXME work-around
    virtual ~_SpuddPolicy() { }

    /// \brief Return the action encoded in the pre-computed policy
    virtual Action getAction(const State& s) override;
    /// \brief Return the action and associated value encoded in the pre-computed policy
    virtual std::tuple<Action,double> getActionValue(const State& s);
};
typedef boost::shared_ptr<_SpuddPolicy> SpuddPolicy;

} // namespace crl

#endif /*SPUDD_HPP_*/
