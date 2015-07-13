/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <gtest/gtest.h>

#include "crl/approx_alp.hpp"
#include "cpputil.hpp"

using namespace std;

///
/// \brief Placeholder for some libDAI experiments
///
TEST(LibDAITest, BasicLibDAITest) {
  auto soln = crl::testing::maxplus_demo();
  EXPECT_EQ(soln, vector<size_t>({1,0,0})) << "max-plus failed";
}

///
/// \brief Test memory layout for libcrl and libDAI
/// \note Main takeaway: sort action variables before state variables in libDAI so that memory layouts match
///
TEST(LibDAITest, FactorMemoryTest) {
  // set up a domain
  Domain domain = boost::make_shared<_Domain>();
  domain->addStateFactor(0, 3, "sf0"); // 4 states
  domain->addStateFactor(0, 2, "sf1"); // 3 states
  domain->addActionFactor(0, 1, "agent1");  // 2 actions
  domain->addActionFactor(0, 4, "agent2");  // 5 actions
  domain->setRewardRange(-1, 0);

  // create flat function
  _FDiscreteFunction<double> f(domain, "test_fn");
  f.addStateFactor(0);
  f.addStateFactor(1);
  f.addActionFactor(0);
  f.addActionFactor(1);
  f.pack();

  // set some random values
  _StateActionIncrementIterator saitr(domain);
  while(saitr.hasNext()) {
      const std::tuple<State,Action>& sa = saitr.next();
      const State& s = std::get<0>(sa);
      const Action& a = std::get<1>(sa);
      f.define(s,a,cpputil::randDouble());
  }

  // create equivalent function (dai::Factor) in libDAI
  const auto& vals = f.values();

  //
  // Main takeaway: sort action variables before state variables in libDIA so that memory layout of crl/dai match
  //

  dai::Var x0(3,4), x1(4,3), a0(0,2), a1(1,5);
  dai::VarSet vs_s(x0, x1);
  dai::VarSet vs_a(a0, a1);
  dai::VarSet vs_all = vs_s | vs_a;
  EXPECT_EQ(vs_all.nrStates(), domain->getNumStates() * domain->getNumActions());
  EXPECT_EQ(vs_all.nrStates(), vals.size());

  dai::Factor f0(vs_all, vals);
  const auto& f0_vals = f0.p();

  // matching storage vectors
  EXPECT_EQ(vals.size(), f0_vals.size());
  EXPECT_EQ(vals, f0_vals.p());

  // test memory layout
  for(dai::State S(vs_all); S.valid(); S++) {
      // note that action variables appear before state variables (due to the chosen labels above)
      LOG_DEBUG(S(x0) << " " << S(x1) << " " << S(a0) << " " << S(a1) << " [" << S << "]: " << S.get());
      // equivalent crl State
      crl::State cs(domain);
      cs.setFactor(0,S(x0));
      cs.setFactor(1,S(x1));
      crl::Action ca(domain);
      ca.setFactor(0,S(a0));
      ca.setFactor(1,S(a1));
      EXPECT_EQ(f0.get(S), f.eval(cs,ca));
  }
}
