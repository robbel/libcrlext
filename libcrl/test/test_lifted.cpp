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
#include "logger.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

namespace {

//
// Test fixtures
//

class LiftedDBNTest : public ::testing::Test {
protected:
    LiftedDBNTest() { }
//  virtual ~LiftedDBNText() { }
    virtual void SetUp() override;
//  virtual void TearDown() override;

    /// \brief Create a rather random simple DBN
    DBN makeSimpleLiftedDBN(Domain domain);

    Domain _domain;
    State  _s;
    DBN    _dbn;
    Indicator<> I1;
    Indicator<> I2;
};

void LiftedDBNTest::SetUp() {
    // set up common domain
    _domain = boost::make_shared<_Domain>();
    _domain->addStateFactor(0, 5, "sf0"); // 4 states
    _domain->addStateFactor(0, 2, "sf1"); // 3 states
    _domain->addStateFactor(0, 2, "sf2"); // 3 states
    _domain->addActionFactor(0, 1, "agent1");  // 2 actions
    _domain->setRewardRange(-1, 0);

    _s = State(_domain, 1);
    // two indicator functions in domain
    I1 = boost::make_shared<_Indicator<>>(_domain);
    I1->addStateFactor(2); // add state factor
    I1->addLiftedFactor(boost::make_shared<_LiftedFactor>(std::initializer_list<Size>{0,1})); // add lifted counter with 3 values
    I1->setState(_s); // only value/index is used TODO enable on `lifted' state
    I2 = boost::make_shared<_Indicator<>>(_domain, SizeVec({0,1,2}), _s);
    ASSERT_TRUE(I1->getSubdomain()->getNumStateFactors() == I1->getStateFactors().size()+I1->getLiftedFactors().size());

    // set up common DBN
    _dbn = makeSimpleLiftedDBN(_domain);
    LOG_DEBUG(*_dbn);
}

DBN LiftedDBNTest::makeSimpleLiftedDBN(Domain domain) {
  DBN dbn = boost::make_shared<_DBN>();

  const RangeVec& ranges = domain->getStateRanges();
  for(Size y = 0; y < domain->getNumStateFactors(); y++) {
      // create a dbn factor
      DBNFactor fa = boost::make_shared<_DBNFactor>(domain, y);
      fa->addDelayedDependency(y); // dependence on self
      fa->addActionDependency(0); // dependence on first action factor
      // add lifted functions
      if(y == 1) {
        fa->addLiftedDependency(boost::make_shared<_LiftedFactor>(std::initializer_list<Size>{0}));
      }
      else if(y == 2) {
        fa->addLiftedDependency(boost::make_shared<_LiftedFactor>(std::initializer_list<Size>{0,1}));
      }
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

TEST_F(LiftedDBNTest, BasicLiftedTest) {
  SUCCEED();
}
