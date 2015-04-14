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
    std::cout << "[DEBUG]: VI planner returned after " << end_time - start_time << "ms" << std::endl;
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

TEST(ALPIntegrationTest, TestSysadmin) {
  srand(time(NULL));

  sysadmin::Sysadmin thesys = buildSysadmin("ring", 1);
  Domain domain = thesys->getDomain();

  FactoredMDP fmdp = thesys->getFactoredMDP();
  std::cout << fmdp->T() << std::endl;

  // create a basis (one indicator per local state, i.e., corresponding to `single' basis in Guestrin thesis)
  FactoredValueFunction fval = boost::make_shared<_FactoredValueFunction>(domain);
  const RangeVec& ranges = domain->getStateRanges();
#if 0
  for(Size fa = 0; fa < ranges.size(); fa++) { // assumption: DBN covers all domain variables
      // place one indicator basis on each possible factor value
      for(Factor fv=0; fv<=ranges[fa].getSpan(); fv++) {
          State dummy_s; // we don't care about actual domain here
          dummy_s.setIndex(ranges[fa].getMin()+fv);
          auto I = boost::make_shared<_Indicator<Reward>>(domain);
          I->addStateFactor(fa);
          I->setState(dummy_s);
          fval->addBasisFunction(I, 0.);
      }
  }
#endif
  for(Size fa = 0; fa < ranges.size(); fa+=2) { // assumption: DBN covers all domain variables
      auto I_o = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa,fa+1}), State(domain,0));
      _StateIncrementIterator sitr(I_o->getSubdomain());
      while(sitr.hasNext()) {
          auto I = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa,fa+1}), sitr.next());
          fval->addBasisFunction(I, 0.);
      }
  }

  // add the constant function
//  auto C  = boost::make_shared<_ConstantFn<Reward>>(domain); // TODO currently not supported, needs Backproj template specialization
  // add some indicators
//  auto I1 = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({0}), State(domain,0));
//  auto I2 = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({2}), State(domain,0));
//  auto I3 = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({4}), State(domain,0));
//  auto I4 = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({6}), State(domain,0));

  // run the ALP planner
  _ALPPlanner planner(fmdp, 0.9);
//  fval->addBasisFunction(C,  0.);
//  fval->addBasisFunction(I1, 0.);
//  fval->addBasisFunction(I2, 0.);
//  fval->addBasisFunction(I3, 0.);
//  fval->addBasisFunction(I4, 0.);

  long start_time = time_in_milli();
  planner.setFactoredValueFunction(fval); // this will be computed
  int res = planner.plan();
  long end_time = time_in_milli();
  std::cout << "[DEBUG]: FALP planner returned after " << end_time - start_time << "ms" << std::endl;

  EXPECT_EQ(res, 0) << "ALP " << (res == 1 ? "generateLP()" : "solve()") << " failed"; // else: lp successfully generated

  if(!res) { // success
    std::cout << "[DEBUG]: Results:" << std::endl;
    for(auto w : fval->getWeight()) {
        std::cout << " W: " << w << std::endl;
    }

    // compare against value iteration solution if we have an exact LP solution (exhaustive basis function set)
    if(fval->getBasis().size() == domain->getNumStates()) {
        std::cout << "[DEBUG]: Comparing with flat VI solution:" << std::endl;

        MDP mdp = convertToMDP(fmdp);
        QTable qt = testVI(mdp, domain, 0.9);
        const vector<double>& wvec = fval->getWeight();
        vector<double>::size_type si = 0;

        _StateIncrementIterator sitr(domain);
        while(sitr.hasNext()) {
          const State& s = sitr.next();
          double r = qt->getV(s);

          EXPECT_TRUE(cpputil::approxEq(r, wvec[si++], 0.1));
          cout << " V(" << s << ")=" << qt->getV(s) << endl;
        }

    }
  }
}
