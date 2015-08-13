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
#include "crl/alp.hpp"
#include "cpputil.hpp"
#include "logger.hpp"

using namespace std;
using namespace crl;

namespace {

//
// Test fixtures
//

class VETest : public ::testing::Test {
protected:
    VETest() { }
//  virtual ~VETest() { }
    virtual void SetUp() override;
//  virtual void TearDown() override;

    /// \brief Create a rather random simple DBN
    FactoredMDP makeFactoredMDP(Domain domain);
    /// \brief Helper function for argMax tests
    /// \return Random test functionals with (only) action dependencies
    FunctionSet<Reward> getActionFunctionals() const;

    Domain _domain;
    FactoredValueFunction _fval;
    ALPPlanner _alp;
};

void VETest::SetUp() {
  // simple_sysadmin_ring_4 domain
  _domain = boost::make_shared<_Domain>();
  _domain->addStateFactor(0, 2);
  _domain->addActionFactor(0, 1, "a1");
  _domain->addStateFactor(0, 2);
  _domain->addActionFactor(0, 1, "a2");
  _domain->addStateFactor(0, 2);
  _domain->addActionFactor(0, 1, "a3");
  _domain->addStateFactor(0, 2);
  _domain->addActionFactor(0, 1, "a4");
  _domain->setRewardRange(0, 4);
  FactoredMDP fmdp = makeFactoredMDP(_domain);

  _fval = boost::make_shared<_FactoredValueFunction>(_domain);
  // add some basis functions
  const RangeVec& ranges = _domain->getStateRanges();
  for(Size fa = 0; fa < ranges.size(); fa++) { // assumption: DBN covers all domain variables
      auto I_o = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa}), State(_domain,0));
      _StateIncrementIterator sitr(I_o->getSubdomain());
      while(sitr.hasNext()) {
          auto I = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa}), sitr.next());
          _fval->addBasisFunction(std::move(I), 0.);
      }
  }

  // adding some value function weights
  vector<double>& vfnweights = _fval->getWeight();
  const auto w = cpputil::ordered_vec(vfnweights.size(),1);
  std::copy(w.begin(), w.end(), vfnweights.begin());
  _alp = boost::make_shared<_ALPPlanner>(fmdp, 0.9);
  _alp->setFactoredValueFunction(_fval);
  _alp->precompute();
}

///
/// \brief Create a rather simple factored MDP
/// FIXME Empty rewards for now..
///
FactoredMDP VETest::makeFactoredMDP(Domain domain) {
  FactoredMDP fmdp = boost::make_shared<_FactoredMDP>(domain);

  const RangeVec& ranges = domain->getStateRanges();
  for(Size y = 0; y < domain->getNumStateFactors(); y++) {
      // create a dbn factor
      DBNFactor fa = boost::make_shared<_DBNFactor>(domain, y);
      fa->addDelayedDependency(y); // dependence on self
      fa->addActionDependency(y);
      fa->pack();
      LRF lrf = boost::make_shared<_LRF>(domain); // empty here
      lrf->addStateFactor(y);
      lrf->addActionFactor(y);
      if(y == 2) {
          lrf->addStateFactor(1); // artificially inflate scope for elimination order tests below
      }
      lrf->pack();

      // fill with random values for transition fn
//    time_t start_time = time_in_milli();
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
//    time_t end_time = time_in_milli();
//    cout << "created DBNFactor in " << end_time - start_time << "ms" << endl;

      // Define LRF
      State dummy_s;
      Action dummy_a;
      dummy_s.setIndex(1);
      dummy_a.setIndex(1);
      lrf->define(State(), Action(), 1000.);
      lrf->define(State(), dummy_a, 2000.);
      lrf->define(dummy_s, dummy_a, -0.75);

      // add random factor to dbn
      fmdp->addDBNFactor(std::move(fa));
      fmdp->addLRF(std::move(lrf));
  }

  return fmdp;
}

FunctionSet<Reward> VETest::getActionFunctionals() const {
  // just as a test, add some action dependencies
  FunctionSet<Reward> F(_domain);
  for(auto& b : _alp->getLRFs()) {
      assert(b->getSubdomain()->getNumStateFactors() != 0);
      F.insert(b);
  }
  // eliminate all state variables
  const SizeVec elim_order_s = cpputil::ordered_vec<Size>(_domain->getNumStateFactors());
  algorithm::variableElimination(F, elim_order_s);

  return F;
}

} // anonymous ns

///
/// \brief Simple variable elimination tests (heuristic vs none)
///
TEST_F(VETest, BasicVETest) {
  // compute factored bellman functionals (1)
  FunctionSet<Reward> F = algorithm::factoredBellmanFunctionals(_domain, _fval);
  const SizeVec elim_order_s = cpputil::ordered_vec<Size>(_domain->getNumStateFactors());
  auto tplBe = algorithm::variableElimination(F, elim_order_s);
  double maxVal = 0.;
  EXPECT_TRUE(F.empty());
  for(const auto& empty_fn : std::get<1>(tplBe)) {
      maxVal += empty_fn->eval(State(),Action());
  }

  // compute factored bellman functionals (2)
  FunctionSet<Reward> F2 = algorithm::factoredBellmanFunctionals(_domain, _fval);
  SizeVec mutable_elim_s(elim_order_s);
  tplBe = algorithm::variableEliminationHeur(F2, mutable_elim_s);
  EXPECT_EQ(elim_order_s, mutable_elim_s);
  double maxVal2 = 0.;
  EXPECT_TRUE(F2.empty());
  for(const auto& empty_fn : std::get<1>(tplBe)) {
      maxVal2 += empty_fn->eval(State(),Action());
  }
  EXPECT_EQ(maxVal, maxVal2);

  // compute factored bellman functionals (3)
  FunctionSet<Reward> F3 = algorithm::factoredBellmanFunctionals(_domain, _fval);
  mutable_elim_s = elim_order_s;
  tplBe = algorithm::variableEliminationHeur(F3, mutable_elim_s, algorithm::ElimHeuristic::MIN_SCOPE);
  EXPECT_NE(elim_order_s, mutable_elim_s);
#if !NDEBUG
  LOG_DEBUG("Chosen elimination ordering by heuristic: ");
  for(auto v : mutable_elim_s) {
    LOG_DEBUG(v);
  }
#endif
  double maxVal3 = 0.;
  EXPECT_TRUE(F3.empty());
  for(const auto& empty_fn : std::get<1>(tplBe)) {
      maxVal3 += empty_fn->eval(State(),Action());
  }
  EXPECT_EQ(maxVal, maxVal3);
  LOG_DEBUG(maxVal3);
}

///
/// \brief Simple argMax variable elimination tests (heuristic vs none)
///
TEST_F(VETest, BasisArgMaxTest) {
  // compute maximizing action (1)
  FunctionSet<Reward> F = getActionFunctionals();
  const SizeVec elim_order_a = cpputil::ordered_vec<Size>(_domain->getNumActionFactors(), _domain->getNumStateFactors());
  tuple<Action,Reward> tpl = algorithm::argVariableElimination(F, elim_order_a);

  Action maxA = std::get<0>(tpl);
  double maxVal = std::get<1>(tpl);
  LOG_DEBUG("argVariableElimination: " << maxA << ": " << maxVal);

  // compute maximizing action (2)
  // Note: actually chosen elimination order is likely same here (all factors depend on one action)
  FunctionSet<Reward> F2 = getActionFunctionals();
  SizeVec mutable_elim_a(elim_order_a);
  tpl = algorithm::argVariableEliminationHeur(F2, mutable_elim_a, algorithm::ElimHeuristic::MIN_SCOPE);

  Action maxA2 = std::get<0>(tpl);
  double maxVal2 = std::get<1>(tpl);
  LOG_DEBUG("argVariableElimination: " << maxA2 << ": " << maxVal2);

  EXPECT_EQ(maxVal, maxVal2);
}
