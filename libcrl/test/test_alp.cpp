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

using namespace std;
using namespace crl;
using namespace cpputil;

///
/// \brief Create a rather random simple FactoredMDP
///
FactoredMDP makeFactoredMDP(Domain domain) {
  FactoredMDP fmdp = boost::make_shared<_FactoredMDP>(domain);

  const RangeVec& ranges = domain->getStateRanges();
  for(Size y = 0; y < domain->getNumStateFactors(); y++) {
      // create a dbn factor
      DBNFactor fa = boost::make_shared<_DBNFactor>(domain, y);
      fa->addDelayedDependency(y); // dependence on self
      fa->addActionDependency(0); // dependence on first action factor
      fa->pack();
      // fill with random values for transition fn

//    time_t start_time = time_in_milli();
      Domain subdomain = fa->getSubdomain();
      for (Size state_index=0; state_index<subdomain->getNumStates(); state_index++) {
              State s(subdomain, state_index);
              for (Size action_index=0; action_index<subdomain->getNumActions(); action_index++) {
                      Action a(subdomain, action_index);
                      //mdp->setR(s, a, randDouble()); // FIXME: empty rewards, for now
                      Probability total = 0.;
                      ProbabilityVec probs(ranges[y].getSpan()+1);
                      for(Factor f=0; f<=ranges[y].getSpan(); f++) { // we don't care about actual factor value here
                          Probability prob = cpputil::randDouble();
                          total += prob;
                          probs[f] = prob;
                      }
                      // normalize
                      for(Factor f=0; f<=ranges[y].getSpan(); f++) {
                          Probability prob = probs[f]/total;
                          fa->setT(s,a,ranges[y].getMin()+f,prob); // here we care about actual factor value
                      }
              }
      }
//    time_t end_time = time_in_milli();
//    cout << "created DBNFactor in " << end_time - start_time << "ms" << endl;

      // add random factor to dbn
      fmdp->addDBNFactor(std::move(fa));
  }

  return fmdp;
}

///
/// \brief Placeholder for some ALP experiments
///
TEST(ALPTest, BasicLpSolveTest) {
  lpsolve::testing::lp_exp();
  int res = lpsolve::testing::lp_demo();
  EXPECT_EQ(res, 0) << "no optimal solution found"; // else: optimal solution found, no error
}

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

///
/// \brief Placeholder for some ALP experiments
///

TEST(ALPTest, BasicALPTest) {
  srand(time(NULL));

  Domain domain = boost::make_shared<_Domain>();
  domain->addStateFactor(0, 3, "sf0"); // 4 states
  domain->addStateFactor(0, 2, "sf1"); // 3 states
  domain->addActionFactor(0, 1, "agent1");  // 2 actions
  domain->setRewardRange(-1, 0);

  FactoredMDP fmdp = makeFactoredMDP(domain);
  std::cout << fmdp->T() << std::endl;
#if 0
  _ALPPlanner planner(fmdp, 0.9);
#endif
  // todo

  SUCCEED();
}

