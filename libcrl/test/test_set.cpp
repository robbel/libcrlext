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
  Indicator I = boost::make_shared<_Indicator>(domain, SizeVec({1}), s2);
  EXPECT_TRUE(!(*I)(s) && (*I)(s2));

  FunctionSet<double> f_set(domain);
  EXPECT_EQ(f_set.getNumFactors(), 0);
  f_set.insert(I);
  EXPECT_EQ(f_set.size(), 1);
  EXPECT_EQ(f_set.getNumFactors(), 1);

  // add another function
  I = boost::make_shared<_Indicator>(domain);
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
  ASSERT_NE(r.first,r.second); // found sth
  ++r.first;
  EXPECT_EQ(r.first,r.second); // exactly one element

  r = f_set.getStateFactor(1);
  ASSERT_NE(r.first,r.second); // found sth
  ++r.first;
  EXPECT_NE(r.first,r.second);
  ++r.first;
  EXPECT_EQ(r.first,r.second); // exactly two elements

  r = f_set.getActionFactor(0);
  ASSERT_NE(r.first,r.second); // found sth
  ++r.first;
  EXPECT_EQ(r.first,r.second); // exactly one element

  r = f_set.getActionFactor(1);
  EXPECT_EQ(r.first,r.second); // found nothing

  f_set.eraseActionFactor(0);
  r = f_set.getActionFactor(0);
  EXPECT_EQ(r.first,r.second); // found nothing

}
