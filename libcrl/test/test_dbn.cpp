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
#include "logger.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

namespace {

//
// Test fixtures
//

class DBNTest : public ::testing::Test {
protected:
    DBNTest() { }
//  virtual ~DBNText() { }
    virtual void SetUp() override;
//  virtual void TearDown() override;

    /// \brief Create a rather random simple DBN
    DBN makeSimpleDBN(Domain domain);

    Domain _domain;
    State  _s;
    State  _s2;
    Action _a;
    DBN    _dbn;
    Indicator<> I1;
    Indicator<> I2;
};

void DBNTest::SetUp() {
    // set up common domain
    _domain = boost::make_shared<_Domain>();
    _domain->addStateFactor(0, 3, "sf0"); // 4 states
    _domain->addStateFactor(0, 2, "sf1"); // 3 states
    _domain->addActionFactor(0, 1, "agent1");  // 2 actions
    _domain->setRewardRange(-1, 0);

    _s = State(_domain, 1);
    _s2 = State(_domain, 2);
    _a = Action(_domain, 0);
    // two indicator functions in domain
    // Note: we can effectively ignore that s2 is not in I's domain: only its index (= 2) will be used by the indicator.
    I1 = boost::make_shared<_Indicator<>>(_domain, SizeVec({1}), _s2); // centered on variable `1' at `s2'
    ASSERT_TRUE(!(*I1)(_s) && (*I1)(_s2));
    I2 = boost::make_shared<_Indicator<>>(_domain);
    I2->addStateFactor(0); // exercise some other code paths
    I2->addStateFactor(1); // an indicator over the complete state factor set
    I2->setState(_s2);
    ASSERT_TRUE(I2->getSubdomain()->getNumStateFactors() == _domain->getNumStateFactors());
    ASSERT_TRUE(!(*I2)(_s) && (*I2)(_s2));

    // set up common DBN
    _dbn = makeSimpleDBN(_domain);
    LOG_INFO(*_dbn);
}

DBN DBNTest::makeSimpleDBN(Domain domain) {
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
      dbn->addDBNFactor(std::move(fa));
  }

  return dbn;
}

} // anonymous ns

///
/// \brief Checks whether 2-TBN is properly normalized
///
TEST_F(DBNTest, DBNNormalizationTest) {
  //srand(time(NULL));

  // normalized?
  Probability total = 0.;
  _StateIncrementIterator si(_domain);
  while(si.hasNext()) {
      total += _dbn->T(_s,_a,si.next());
  }
  EXPECT_DOUBLE_EQ(1., total);

  // alternative invokation 1 (template specialization)
  total = 0.;
  si.reset();
  while(si.hasNext()) {
      total += _dbn->T(_s,_a,si.next(),identity_map,identity_map,identity_map);
  }
  EXPECT_DOUBLE_EQ(1., total);

  // alternative invokation 2 (with mapping)
  total = 0.;
  const subdom_map mapping(cpputil::ordered_vec<Size>(_domain->getNumStateFactors()));
  si.reset();
  while(si.hasNext()) {
      total += _dbn->T(_s,_a,si.next(),mapping,mapping,mapping);
  }
  EXPECT_DOUBLE_EQ(1.,total);
}

///
/// \brief Some backprojection tests of the indicator functions
/// \see the integration_testing package for more complex backprojection tests
/// \todo Test with invalid functions too (and expect exception)
///
TEST_F(DBNTest, DBNBackprojectionTests) {
  _Backprojection<double> B(_domain, *_dbn, I1, "bp1");
  EXPECT_TRUE(B.getStateFactors().size() == 1 && B.getActionFactors().size() == 1); // based on DBN structure above
  EXPECT_TRUE(B.getStateFactors()[0] == 1);

  // compute subdomain and cache values
  B.cache();
  EXPECT_TRUE(B.getSubdomain()->getNumStateFactors() == 1 && B.getSubdomain()->getNumActionFactors() == 1); // based on DBN structure above

  // test whether backprojection resulted in some meaningful values, given DBN structure above
  DBNFactor fa = _dbn->factor(1);
  const State empty_s;
  const Size indicated_state = I1->getStateIndex();
  Domain subdomain = fa->getSubdomain(); // TODO enable StateActionIncrementIterator here too!
  for (Size state_index=0; state_index<subdomain->getNumStates(); state_index++) {
          State s(subdomain, state_index);
          for (Size action_index=0; action_index<subdomain->getNumActions(); action_index++) {
                  Action a(subdomain, action_index);
                  double v_dbn = fa->T(s, empty_s, a, indicated_state);
                  EXPECT_DOUBLE_EQ(v_dbn, B(s,a));
          }
  }

  // A backprojection with an indicator over complete state factor scope (2 state factors)
  _Backprojection<double> B2(_domain, *_dbn, I2, "bp2");
  EXPECT_TRUE(B2.getStateFactors().size() == 2 && B2.getActionFactors().size() == 1); // based on DBN structure above
  EXPECT_TRUE(B2.getStateFactors()[0] == 0 && B2.getStateFactors()[1] == 1);

  B2.cache();
  EXPECT_TRUE(B2.getSubdomain()->getNumStateFactors() == 2 && B2.getSubdomain()->getNumActionFactors() == 1); // based on DBN structure above

  // Backprojection is now the complete dbn
  const subdom_map mapping(cpputil::ordered_vec<Size>(_domain->getNumStateFactors()));
  for (Size state_index=0; state_index<_domain->getNumStates(); state_index++) {
          State s(_domain, state_index);
          for (Size action_index=0; action_index<_domain->getNumActions(); action_index++) {
                  Action a(_domain, action_index);
                  double v_dbn = _dbn->T(s, a, _s2);
                  EXPECT_DOUBLE_EQ(v_dbn, B2(s,a));
                  // alternative means of computing the same thing
                  v_dbn = _dbn->T(s, a, _s2, mapping, mapping, subdom_map(cpputil::ordered_vec<Size>(_domain->getNumActionFactors())));
                  EXPECT_DOUBLE_EQ(v_dbn, B2(s,a));
          }
  }

  //
  // Now two functions have been backprojected through DBN. Verify whether expectations are computed correctly.
  //

  // compute expectation over all successor states in domain
  double sum = 0.;
  const State s0(_domain, 0); // an arbitrary starting state
  const Action a0(_domain,0);
  _StateIncrementIterator sitr(_domain);
  while(sitr.hasNext()) {     // over all successor states
      const State& s = sitr.next();
      double p_dbn = _dbn->T(s0,a0,s);
      State ms = I1->mapState(s);
      const double v1 = (*I1)(ms,Action());
      ms = I2->mapState(s);
      const double v2 = (*I2)(ms,Action());
      sum += p_dbn * (v1+v2);
  }

  // compare to sum over backprojections
  double bsum = 0.;
  State ms = B.mapState(s0);
  Action ma = B.mapAction(a0);
  bsum += B(ms,ma);
  ms = B2.mapState(s0);
  ma = B2.mapAction(a0);
  bsum += B2(ms,ma);
  EXPECT_DOUBLE_EQ(sum, bsum);
}

///
/// \brief Some basic ``logic'' tests
///
TEST_F(DBNTest, BasicLogicTests) {
  _Domain orig = *(I2->getSubdomain());
  I2->addStateFactor(1); // insert an existing one again
  _Domain mod = *(I2->getSubdomain());
  bool var = orig.getStateIndexComponents() == mod.getStateIndexComponents();
  EXPECT_TRUE(var);

  I2->eraseStateFactor(0);
  I2->computeSubdomain(); // compute new subdomain
  mod = *(I2->getSubdomain());
  var = orig.getStateIndexComponents() == mod.getStateIndexComponents(); // subdomain changed
  EXPECT_FALSE(var);
  const RangeVec& domReg = _domain->getStateRanges();
  var = mod.getNumStates() == domReg[1].getSpan()+1; // number of states in new subdomain is correct
  EXPECT_TRUE(var);
}
