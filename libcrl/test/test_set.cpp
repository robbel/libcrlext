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

using namespace std;
using namespace crl;
using namespace cpputil;

///
/// \brief Placeholder for some initial experiments
///
TEST(FunctionSetTest, BasicTest) {
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
  // Test FunctionSet with a couple of functions
  //

  // Note: we can effectively ignore that s is not in I's domain: only its index (= 2) will be used by the indicator.
  Indicator<> I = boost::make_shared<_Indicator<>>(domain, SizeVec({1}), s2);
  EXPECT_TRUE(!(*I)(s) && (*I)(s2));
  EXPECT_DOUBLE_EQ(algorithm::sum_over_domain(I.get(),false), 1.);

  FunctionSet<double> f_set(domain);
  EXPECT_EQ(f_set.getNumFactors(), 0);
  f_set.insert(I);
  EXPECT_EQ(f_set.size(), 1);
  EXPECT_EQ(f_set.getNumFactors(), 1);

  // add another function
  I = boost::make_shared<_Indicator<>>(domain);
  I->addStateFactor(0); // exercise some other code paths
  I->addStateFactor(1);
  I->setState(s2);
  f_set.insert(I);
  EXPECT_EQ(f_set.size(), 3);
  EXPECT_EQ(f_set.getNumFactors(), 2);

  // add another function (only one action dependency)
  EmptyFunction<double> D = boost::make_shared<_EmptyFunction<double>>(domain);
  D->addActionFactor(0);
  f_set.insert(D);
  EXPECT_EQ(f_set.size(), 4);
  EXPECT_EQ(f_set.getNumFactors(), 3);

  //
  // Test FunctionSet retrieval
  //

  using range = FunctionSet<double>::range;
  range r = f_set.getStateFactor(0);
  ASSERT_TRUE(r.hasNext()); // found sth
  r.next();
  EXPECT_TRUE(!r.hasNext()); // exactly one element

  r = f_set.getStateFactor(1);
  ASSERT_TRUE(r.hasNext()); // found sth
  r.next();
  EXPECT_TRUE(r.hasNext());
  r.next();
  EXPECT_TRUE(!r.hasNext()); // exactly two elements

  r = f_set.getActionFactor(0);
  ASSERT_TRUE(r.hasNext()); // found sth
  const DiscreteFunction<double>& f = r.next();
  EXPECT_THROW(f->eval(s,a), cpputil::InvalidException); // should have returned EmptyFunction
  EXPECT_TRUE(!r.hasNext()); // exactly one element

  r = f_set.getActionFactor(1);
  EXPECT_TRUE(!r.hasNext()); // found nothing

  f_set.eraseActionFactor(0);
  r = f_set.getActionFactor(0);
  EXPECT_TRUE(!r.hasNext()); // found nothing
}

///
/// \brief Some basic operations on functions
///
TEST(FunctionSetTest, FunctionAdditionTest) {
  Domain domain = boost::make_shared<_Domain>();
  domain->addStateFactor(0, 3, "sf0"); // 4 states
  domain->addStateFactor(0, 2, "sf1"); // 3 states
  domain->addActionFactor(0, 1, "agent1");  // 2 actions
  domain->setRewardRange(-1, 0);

  const State s(domain, 1);
  const State s2(domain,2);
  const Action a(domain,0);

  _FDiscreteFunction<double> f(domain, "test_name");
  f.addStateFactor(0);
  f.addStateFactor(1);
  f.addActionFactor(0);
  _FDiscreteFunction<double> f2(domain, "test_name");
  f2.addStateFactor(0);
  f2.addStateFactor(1);
  f2.addActionFactor(0);

  f.pack();
  f2.pack();

  f.define(s,a, 3);
  EXPECT_EQ(f.eval(s,a), 3);
  EXPECT_DOUBLE_EQ(algorithm::sum_over_domain(&f,false), 3.);
  EXPECT_DOUBLE_EQ(algorithm::sum_over_domain(&f,true), 3.);
  f2.define(s2,a,4);
  EXPECT_EQ(f2.eval(s2,a), 4);

  f += f2;
  EXPECT_EQ(f.eval(s,a), 3);
  EXPECT_EQ(f.eval(s2,a), 4);

  f *= 2;
  EXPECT_EQ(f.eval(s,a), 6);
  EXPECT_EQ(f.eval(s2,a), 8);
  EXPECT_DOUBLE_EQ(algorithm::sum_over_domain(&f,false), 14.);
  EXPECT_DOUBLE_EQ(algorithm::sum_over_domain(&f,true), 14.);

  //
  // examples where f2 has reduced scope and is subtracted from f
  //

  f2.eraseStateFactor(0);
  f2.pack();
  EXPECT_THROW(f += f2, cpputil::InvalidException); // f2 still contains action scope, not allowed as per current implementation

  f2.eraseActionFactor(0);
  f2.pack();
  f2.define(s,Action(),5);
  f2.define(s2,Action(),3);
  // print before
  std::cout << "Before: " << std::endl;
  _StateIncrementIterator sitr(f.getSubdomain());
  _ActionIncrementIterator aitr(f.getSubdomain());
  while(sitr.hasNext()) {
      const State& s = sitr.next();
      aitr.reset();
      while(aitr.hasNext()) {
          const Action& a = aitr.next();
          std::cout << s << "," << a << ": " << f(s,a) << std::endl;
      }
  }
  EXPECT_NO_THROW(f -= f2); // f2's domain is proper subset
  std::cout << "After: " << std::endl;
  sitr.reset();
  while(sitr.hasNext()) {
      const State& s = sitr.next();
      aitr.reset();
      while(aitr.hasNext()) {
          const Action& a = aitr.next();
          std::cout << s << "," << a << ": " << f(s,a) << std::endl;
      }
  }

  f.eraseStateFactor(1);
  f.pack();
  EXPECT_THROW(f += f2, cpputil::InvalidException); // f2's domain is not a proper subset anymore
}
