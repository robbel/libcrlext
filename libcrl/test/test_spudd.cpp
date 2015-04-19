/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <gtest/gtest.h>

#include "crl/conversions.hpp"
#include "crl/factor_learner.hpp"
#include "crl/spudd.hpp"
#include "cpputil.hpp"
#include "logger.hpp"

using namespace std;
using namespace crl;

namespace {

///
/// \brief Create a rather simple factored MDP
/// FIXME Empty rewards for now..
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

} // anonymous ns

///
/// \brief Simple FMDP construction/export test
///
TEST(SPUDDTest, BasicTest) {
  try {
    srand(0);

    Domain domain = boost::make_shared<_Domain>();
    domain->addStateFactor(0, 299, "first_state"); // 300 states
    domain->addActionFactor(0, 4, "first_agent");  // 5 actions
    domain->setRewardRange(-1, 0);

    FactoredMDP fmdp = makeFactoredMDP(domain);

    EXPECT_EQ(exportToSpudd(fmdp, domain, 0.99, "test", "test.spudd"), 0);

  }
  catch(const cpputil::Exception& e) {
    cerr << e << endl;
    FAIL();
  }

  SUCCEED();
}

///
/// \brief Simple Policy Querying Agent test
///
TEST(SPUDDTest, PolicyQueryTest) {
    // sysadmin_ring_4 domain
    Domain sysdomain = boost::make_shared<_Domain>();
    sysdomain->addStateFactor(0, 2);
    sysdomain->addStateFactor(0, 2);
    sysdomain->addActionFactor(0, 1, "a1");
    sysdomain->addStateFactor(0, 2);
    sysdomain->addStateFactor(0, 2);
    sysdomain->addActionFactor(0, 1, "a2");
    sysdomain->addStateFactor(0, 2);
    sysdomain->addStateFactor(0, 2);
    sysdomain->addActionFactor(0, 1, "a3");
    sysdomain->addStateFactor(0, 2);
    sysdomain->addStateFactor(0, 2);
    sysdomain->addActionFactor(0, 1, "a4");
    _SpuddPolicy sp(sysdomain, "SPUDD-OPTDual.ADD");

    Action aopt = sp.getAction(State(sysdomain,74));
    LOG_DEBUG(aopt);

    EXPECT_EQ(aopt.getIndex(), 3);
}


