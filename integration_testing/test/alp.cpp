/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <gtest/gtest.h>

#include "crl/alp.hpp"
#include "crl/alp_lpsolve.hpp"
#include "crl/env_sysadmin.hpp"
#include "crl/vi.hpp"
#include "logger.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

//
// More complex integration test that runs the ALP solver on the SysAdmin environment
//

namespace {

///
/// \brief Run value-iteration on flat MDP
///
QTable testVI(MDP mdp, Domain domain, float gamma) {
    long start_time = time_in_milli();
    VIPlanner planner(new _FlatVIPlanner(domain, mdp, .0001, gamma));
    planner->plan();
    long end_time = time_in_milli();
    LOG_INFO("VI planner returned after " << end_time - start_time << "ms");
    QTable qtable = planner->getQTable();
    return qtable;
}

///
/// \brief Convert factored MDP to flat MDP (exhaustive joint S,A enumeration)
///
MDP convertToMDP(FactoredMDP fmdp) {
  const Domain& domain = fmdp->getDomain();
  _FMDP mdp(domain);

  for (Size state_index=0; state_index<domain->getNumStates(); state_index++) {
          State s(domain, state_index);
          for (Size action_index=0; action_index<domain->getNumActions(); action_index++) {
                  Action a(domain, action_index);
                  mdp.setR(s, a, fmdp->R(s, a));
                  for (Size next_index=0; next_index<domain->getNumStates(); next_index++) {
                          State n(domain, next_index);
                          Probability p = fmdp->T(s,a,n);
                          mdp.setT(s, a, n, p);
                  }
          }
  }

  return boost::make_shared<_FMDP>(mdp);
}

} // anonymous ns

TEST(ALPIntegrationTest, TestSysadminExhaustiveBasis) {
  srand(time(NULL));

  sysadmin::Sysadmin thesys = buildSysadmin("ring", 1);
  Domain domain = thesys->getDomain();

  FactoredMDP fmdp = thesys->getFactoredMDP();
  LOG_INFO(fmdp->T());

#if 0
  // debug print the CPTs
  for(Size i = 0; i < fmdp->T().size(); i++) {
      const auto& fa = fmdp->T().factor(i);
      LOG_DEBUG(*fa << " CPT: ");
      _StateActionIncrementIterator saitr(fa->getSubdomain());
      while(saitr.hasNext()) {
          const std::tuple<State,Action>& sa = saitr.next();
          const State& s = std::get<0>(sa);
          const Action& a = std::get<1>(sa);
          LOG_DEBUG(" " << s << " " << a << ": " << (*fa)(s,a));
      }
  }
#endif

  FactoredValueFunction fval = boost::make_shared<_FactoredValueFunction>(domain);
  // add the constant basis (to guarantee LP feasibility)
//  auto cfn = boost::make_shared<_ConstantFn<Reward>>(domain);
//  fval->addBasisFunction(std::move(cfn), 0.);
#if 0
  // add more basis functions
  const RangeVec& ranges = domain->getStateRanges();
  // create a basis (one indicator per local state, i.e., corresponding to `single' basis in Guestrin thesis)
  for(Size fa = 0; fa < ranges.size(); fa++) { // assumption: DBN covers all domain variables
      // place one indicator basis on each possible factor value
      for(Factor fv=0; fv<=ranges[fa].getSpan(); fv++) {
          State dummy_s; // we don't care about actual domain here
          dummy_s.setIndex(ranges[fa].getMin()+fv);
          auto I = boost::make_shared<_Indicator<Reward>>(domain);
          I->addStateFactor(fa);
          I->setState(dummy_s);
          fval->addBasisFunction(std::move(I), 0.);
      }
  }
#endif
#if 0
  for(Size fa = 0; fa < ranges.size(); fa+=2) { // over all status variables
    auto I = boost::make_shared<_Indicator<Reward>>(domain);
    I->addStateFactor(fa);
    State dummy_s; // we don't care about actual domain here
    dummy_s.setIndex((Factor)sysadmin::Status::GOOD);
    I->setState(dummy_s);
    fval->addBasisFunction(std::move(I), 0.);
  }
#endif
  // exhaustive indicator basis
  _StateIncrementIterator sitr(domain);
  while(sitr.hasNext()) {
      auto I = boost::make_shared<_Indicator<Reward>>(domain, cpputil::ordered_vec<Size>(domain->getNumStateFactors()), sitr.next());
      fval->addBasisFunction(std::move(I), 0.);
  }
#if 0
  for(Size fa = 0; fa < ranges.size(); fa+=2) { // assumption: DBN covers all domain variables
      auto I_o = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa,fa+1}), State(domain,0));
      _StateIncrementIterator sitr(I_o->getSubdomain());
      while(sitr.hasNext()) {
          auto I = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa,fa+1}), sitr.next());
          fval->addBasisFunction(std::move(I), 0.);
      }
  }
#endif

  // run the ALP planner
  _ALPPlanner planner(fmdp, 0.9);

  long start_time = time_in_milli();
  planner.setFactoredValueFunction(fval); // this will be computed
  int res = planner.plan();
  long end_time = time_in_milli();
  LOG_INFO("FALP planner returned after " << end_time - start_time << "ms");

  EXPECT_EQ(res, 0) << "ALP " << (res == 1 ? "generateLP()" : "solve()") << " failed"; // else: lp successfully generated

  if(!res) { // success
    LOG_INFO("Results:");
    for(auto w : fval->getWeight()) {
        LOG_INFO(" W: " << w);
    }

    // compare against value iteration solution if we have an exact LP solution (exhaustive basis function set)
    if(fval->getBasis().size() == domain->getNumStates()) {
        LOG_INFO("Comparing with flat VI solution:");

        MDP mdp = convertToMDP(fmdp);
        QTable qt = testVI(mdp, domain, 0.9);
        const vector<double>& wvec = fval->getWeight();
        vector<double>::size_type si = 0;

        _StateIncrementIterator sitr(domain);
        while(sitr.hasNext()) {
          const State& s = sitr.next();
          double r = qt->getV(s);

          EXPECT_TRUE(cpputil::approxEq(r, wvec[si++], 0.1));
          EXPECT_TRUE(cpputil::approxEq(r, (double)fval->getV(s), 0.1));
          LOG_INFO(" V(" << s << ")=" << qt->getV(s));
        }
    }

    // compute maximum value for a specific state
    State js(domain, 1);
    // eliminate only action variables
    SizeVec elim_order = cpputil::ordered_vec<Size>(domain->getNumActionFactors(), domain->getNumStateFactors());
    tuple<Action,Reward> res = fval->getBestAction(js, elim_order);
    LOG_INFO("Maximum value in " << js << " after variable elimination: " << std::get<1>(res));
    // for an exhaustive basis function set we can compare result with known V-fn
    if(fval->getBasis().size() == domain->getNumStates()) {
      EXPECT_TRUE(cpputil::approxEq(std::get<1>(res), fval->getV(js), Reward(0.1)));
    }
  }
}
