/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <gtest/gtest.h>

#include "crl/factor_learner.hpp"
#include "crl/alp.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

///
/// \brief Create a rather random DBN
///
DBN makeDBN(Domain domain) {
  DBN dbn = boost::make_shared<_DBN>();

  const RangeVec& ranges = domain->getStateRanges();
  for(Size y = 0; y < domain->getNumStateFactors(); y++) {
      // create a dbn factor
      DBNFactor fa = boost::make_shared<_DBNFactor>(domain, y);
      fa->addDelayedDependency(y); // dependence on self
      fa->addActionDependency(0); // dependence on first action factor
      fa->pack();
      // fill with random values for transition fn

      time_t start_time = time_in_milli();
      Domain subdomain = fa->getSubdomain();
      for (Size state_index=0; state_index<subdomain->getNumStates(); state_index++) {
              State s(subdomain, state_index);
              for (Size action_index=0; action_index<subdomain->getNumActions(); action_index++) {
                      Action a(subdomain, action_index);
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
      time_t end_time = time_in_milli();
      cout << "created DBNFactor " << y << " in " << end_time - start_time << "ms" << endl;

      // add random factor to dbn
      dbn->addDBNFactor(fa);
  }

  return dbn;
}

///
/// \brief Placeholder for some initial experiments
///
TEST(DBNTest, BasicTest) {
  srand(time(NULL));

  Domain domain = boost::make_shared<_Domain>();
  domain->addStateFactor(0, 3, "sf0"); // 4 states
  domain->addStateFactor(0, 2, "sf1"); // 3 states
  domain->addActionFactor(0, 1, "agent1");  // 2 actions
  domain->setRewardRange(-1, 0);

  const State s(domain, 1);
  const State s2(domain,2);
  const Action a(domain, 0);

  //
  // Generate indicator function on sf1 = 1
  //

  // Note: we can effectively ignore that s is not in I's domain: only its index (= 2) will be used by the indicator.
  Indicator I = boost::make_shared<_Indicator>(domain, SizeVec({1}), s2);
  EXPECT_TRUE(!(*I)(s) && (*I)(s2));

  //
  // Basic DBN checks
  //

  DBN dbn = makeDBN(domain);

  // normalized?
  Probability total = 0.;
  _StateIncrementIterator si(domain);
  while(si.hasNext()) {
      total += dbn->T(s,a,si.next());
  }
  EXPECT_DOUBLE_EQ(1.,total);

  // alternative invokation 1 (template specialization)
  total = 0.;
  si.reset();
  while(si.hasNext()) {
      total += dbn->T(s,a,si.next(),identity_map,identity_map,identity_map);
  }
  EXPECT_DOUBLE_EQ(1.,total);

  // alternative invokation 2 (with mapping)
  total = 0.;
  const inverse_map<Size> mapping(cpputil::ordered_vec<Size>(domain->getNumStateFactors()));
  si.reset();
  while(si.hasNext()) {
      total += dbn->T(s,a,si.next(),mapping,mapping,mapping);
  }
  EXPECT_DOUBLE_EQ(1.,total);

  //
  // Test backprojection of indicator through DBN
  //

  // TODO test with invalid functions too (and expect exception)
  // TODO test with indicator of increased state factor scope
  // TODO increase complexity of another DBN2 (perhaps even randomize parent dependencies in (s,a))

  _Backprojection<double> B(domain, dbn, I, "backproject_thru_dbn");
  EXPECT_TRUE(B.getStateFactors().size() == 1 && B.getActionFactors().size() == 1); // based on DBN structure above
  EXPECT_TRUE(B.getStateFactors()[0] == 1);

  // compute subdomain and cache values
  B.cache();
  EXPECT_TRUE(B.getSubdomain()->getNumStateFactors() == 1 && B.getSubdomain()->getNumActionFactors() == 1); // based on DBN structure above

  // test whether backprojection resulted in some meaningful values, given DBN structure above
  DBNFactor fa = dbn->factor(1);
  const State empty_s;
  const Size indicated_state = I->getStateIndex();
  Domain subdomain = fa->getSubdomain(); // TODO enable StateActionIncrementIterator here too!
  for (Size state_index=0; state_index<subdomain->getNumStates(); state_index++) {
          State s(subdomain, state_index);
          for (Size action_index=0; action_index<subdomain->getNumActions(); action_index++) {
                  Action a(subdomain, action_index);
                  double v_dbn = fa->T(s, empty_s, a, indicated_state);
                  EXPECT_DOUBLE_EQ(v_dbn, B(s,a));
          }
  }

}
