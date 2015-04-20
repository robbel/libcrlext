/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <gtest/gtest.h>

#include "crl/function.hpp"
#include "logger.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

//
// Test fixtures
//

class FunctionSetTest : public ::testing::Test {
protected:
    FunctionSetTest() { }
//  virtual ~DBNText() { }
    virtual void SetUp() override;
//  virtual void TearDown() override;

    /// \brief Insert some functions into a \a FunctionSet
    boost::shared_ptr<FunctionSet<double>> insertFunctionTest();

    Domain _domain;
    State  _s;
    State  _s2;
    Action _a;
    Indicator<> I1;
    Indicator<> I2;
};

void FunctionSetTest::SetUp() {
    // set up common domain
    _domain = boost::make_shared<_Domain>();
    _domain->addStateFactor(0, 3, "sf0"); // 4 states
    _domain->addStateFactor(0, 2, "sf1"); // 3 states
    _domain->addActionFactor(0, 1, "agent1");  // 2 actions
    _domain->setRewardRange(-1, 0);

    _s = State(_domain, 1);
    _s2 = State(_domain, 2);
    _a = Action(_domain, 0);
}

boost::shared_ptr<FunctionSet<double>> FunctionSetTest::insertFunctionTest() {
    boost::shared_ptr<FunctionSet<double>> f_set = boost::make_shared<FunctionSet<double>>(_domain);

    // Note: we can effectively ignore that s is not in I's domain: only its index (= 2) will be used by the indicator.
    Indicator<> I = boost::make_shared<_Indicator<>>(_domain, SizeVec({1}), _s2);
    EXPECT_TRUE(!(*I)(_s) && (*I)(_s2));
    EXPECT_DOUBLE_EQ(algorithm::sum_over_domain(I.get(),false), 1.);

    EXPECT_EQ(f_set->getNumFactors(), 0);
    f_set->insert(I);
    EXPECT_EQ(f_set->size(), 1);
    EXPECT_EQ(f_set->getNumFactors(), 1);

    // add another function
    I = boost::make_shared<_Indicator<>>(_domain);
    I->addStateFactor(0); // exercise some other code paths
    I->addStateFactor(1);
    I->setState(_s2);
    f_set->insert(I);
    EXPECT_EQ(f_set->size(), 3);
    EXPECT_EQ(f_set->getNumFactors(), 2);

    // add another function (only one action dependency)
    EmptyFunction<double> D = boost::make_shared<_EmptyFunction<double>>(_domain);
    D->addActionFactor(0);
    f_set->insert(D);
    EXPECT_EQ(f_set->size(), 4);
    EXPECT_EQ(f_set->getNumFactors(), 3);

    return f_set;
}

///
/// \brief Test \a FunctionSet with a couple of functions
///
TEST_F(FunctionSetTest, BasicInsertionTests) {
    // fill FunctionSet
    ASSERT_TRUE(insertFunctionTest() != nullptr);
}

///
/// \brief Test \a FunctionSet retrieval
///
TEST_F(FunctionSetTest, BasicRetrievalTests) {
  // fill FunctionSet again
  const auto& f_set = insertFunctionTest();
  using range = FunctionSet<double>::range;

  range r = f_set->getStateFactor(0);
  ASSERT_TRUE(r.hasNext()); // found sth
  r.next();
  EXPECT_TRUE(!r.hasNext()); // exactly one element

  r = f_set->getStateFactor(1);
  ASSERT_TRUE(r.hasNext()); // found sth
  r.next();
  EXPECT_TRUE(r.hasNext());
  r.next();
  EXPECT_TRUE(!r.hasNext()); // exactly two elements

  r = f_set->getActionFactor(0);
  ASSERT_TRUE(r.hasNext()); // found sth
  const DiscreteFunction<double>& f = r.next();
  EXPECT_THROW(f->eval(_s,_a), cpputil::InvalidException); // should have returned EmptyFunction
  EXPECT_TRUE(!r.hasNext()); // exactly one element

  r = f_set->getActionFactor(1);
  EXPECT_TRUE(!r.hasNext()); // found nothing

  f_set->eraseActionFactor(0);
  r = f_set->getActionFactor(0);
  EXPECT_TRUE(!r.hasNext()); // found nothing
}

///
/// \brief Some basic operations on functions
///
TEST_F(FunctionSetTest, FunctionAdditionTest) {
  _FDiscreteFunction<double> f(_domain, "test_name");
  f.addStateFactor(0);
  f.addStateFactor(1);
  f.addActionFactor(0);
  _FDiscreteFunction<double> f2(_domain, "test_name");
  f2.addStateFactor(0);
  f2.addStateFactor(1);
  f2.addActionFactor(0);

  f.pack();
  f2.pack();

  f.define(_s,_a, 3);
  EXPECT_EQ(f.eval(_s,_a), 3);
  EXPECT_DOUBLE_EQ(algorithm::sum_over_domain(&f,false), 3.);
  EXPECT_DOUBLE_EQ(algorithm::sum_over_domain(&f,true), 3.);
  f2.define(_s2,_a,4);
  EXPECT_EQ(f2.eval(_s2,_a), 4);

  f += f2;
  EXPECT_EQ(f.eval(_s,_a), 3);
  EXPECT_EQ(f.eval(_s2,_a), 4);

  f -= f2;
  EXPECT_EQ(f.eval(_s,_a), 3);
  EXPECT_EQ(f.eval(_s2,_a), 0);

  f *= 2;
  EXPECT_EQ(f.eval(_s,_a), 6);
  EXPECT_EQ(f.eval(_s2,_a), 0);
  EXPECT_DOUBLE_EQ(algorithm::sum_over_domain(&f,false), 6.);
  EXPECT_DOUBLE_EQ(algorithm::sum_over_domain(&f,true), 6.);

  //
  // examples where f2 has reduced scope and is subtracted from f
  //

  f2.eraseStateFactor(0);
  f2.pack();
  EXPECT_THROW(f += f2, cpputil::InvalidException); // f2 still contains action scope, not allowed as per current implementation

  f2.eraseActionFactor(0); // new scope is just over state factor `1'
  f2.pack();
  f2.define(_s,Action(),5);  // sf1(1) = 5
  f2.define(_s2,Action(),3); // sf1(2) = 3
  // change function f to see results of operation
  State t(f.getSubdomain());
  t.setFactor(1,2);
  f.define(t,_a,20.);
  // print before
  LOG_INFO("Function f before f-=f2: ");
  _StateIncrementIterator sitr(f.getSubdomain());
  _ActionIncrementIterator aitr(f.getSubdomain());
  while(sitr.hasNext()) {
      const State& s = sitr.next();
      aitr.reset();
      while(aitr.hasNext()) {
          const Action& a = aitr.next();
          LOG_INFO(" " << s << "," << a << ": " << f(s,a));
      }
  }
  EXPECT_NO_THROW(f -= f2); // f2's domain is proper subset
  LOG_INFO("Function f after f-=f2: ");
  sitr.reset();
  while(sitr.hasNext()) {
      const State& s = sitr.next();
      aitr.reset();
      while(aitr.hasNext()) {
          const Action& a = aitr.next();
          LOG_INFO(" " << s << "," << a << ": " << f(s,a));
      }
  }
  EXPECT_DOUBLE_EQ(f(t,_a), 20.-3.);

  f.eraseStateFactor(1);
  f.pack();
  EXPECT_THROW(f += f2, cpputil::InvalidException); // f2's domain is not a proper subset anymore
}

///
/// \brief Function copying
///
TEST_F(FunctionSetTest, FunctionCopyTest) {
  _FDiscreteFunction<double> f(_domain);
  f.addStateFactor(0);
  f.addStateFactor(1);
  f.addActionFactor(0);
  _FDiscreteFunction<double> f2(_domain);

  f.pack();

  f.define(_s,_a, 3);

  // copy assignment
  f2 = f;
  EXPECT_EQ(f(_s,_a), f2(_s,_a));

  f2 += f2;
  ASSERT_NE(f(_s,_a), f2(_s,_a)); // we are working with a copy
  EXPECT_DOUBLE_EQ(f2(_s,_a), 2*f(_s,_a));

  _FDiscreteFunction<double> f3(std::move(f2));
  EXPECT_DOUBLE_EQ(f3(_s,_a), 2*f(_s,_a));
}

///
/// \brief Function instantiation in a particular state
///
TEST_F(FunctionSetTest, FunctionInstantiationTest) {
  // test with FDiscreteFunction
  _FDiscreteFunction<double> f(_domain);
  f.addStateFactor(0);
  f.addStateFactor(1);
  f.addActionFactor(0);
  f.pack();
  f.define(_s,_a, 3);
  f.define(_s2,_a, 4);

  // the resulting function only depends on action factors
  const DiscreteFunction<double>& rf = algorithm::instantiate(&f, _s, false);
  EXPECT_DOUBLE_EQ(rf->eval(_a), 3.);

  // test with an Indicator, should yield empty scope
  Indicator<> I = boost::make_shared<_Indicator<>>(_domain, SizeVec({1}), _s2);
  const DiscreteFunction<double>& rf2 = algorithm::instantiate(I.get(), _s2, false);
  EXPECT_TRUE(rf2->getSubdomain()->getNumStateFactors() == 0 && rf2->getSubdomain()->getNumActionFactors() == 0);
  // if instantiated at _s2, the value should be 1
  EXPECT_EQ(rf2->eval(Action()), 1);

  // instantiate same indicator on a different state, should yield 0
  const DiscreteFunction<double>& rf3 = algorithm::instantiate(I.get(), _s, false);
  EXPECT_TRUE(rf3->getSubdomain()->getNumStateFactors() == 0 && rf3->getSubdomain()->getNumActionFactors() == 0);
  EXPECT_EQ(rf3->eval(Action()), 0);
}
