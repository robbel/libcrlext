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
      fa->addActionDependency(y);
      fa->pack();
      LRF lrf = boost::make_shared<_LRF>(domain); // empty here
      lrf->addStateFactor(y);
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

      // add random factor to dbn
      fmdp->addDBNFactor(std::move(fa));
      fmdp->addLRF(std::move(lrf));
  }

  return fmdp;
}

} // anonymous ns

///
/// \brief Simple variable elimination tests (heuristic vs none)
///
TEST(VETest, BasicVETest) {
  // simple_sysadmin_ring_4 domain
  Domain domain = boost::make_shared<_Domain>();
  domain->addStateFactor(0, 2);
  domain->addActionFactor(0, 1, "a1");
  domain->addStateFactor(0, 2);
  domain->addActionFactor(0, 1, "a2");
  domain->addStateFactor(0, 2);
  domain->addActionFactor(0, 1, "a3");
  domain->addStateFactor(0, 2);
  domain->addActionFactor(0, 1, "a4");
  domain->setRewardRange(0, 4);
  FactoredMDP fmdp = makeFactoredMDP(domain);

  FactoredValueFunction fval = boost::make_shared<_FactoredValueFunction>(domain);
  // add some basis functions
  const RangeVec& ranges = domain->getStateRanges();
  for(Size fa = 0; fa < ranges.size(); fa++) { // assumption: DBN covers all domain variables
      auto I_o = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa}), State(domain,0));
      _StateIncrementIterator sitr(I_o->getSubdomain());
      while(sitr.hasNext()) {
          auto I = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa}), sitr.next());
          fval->addBasisFunction(std::move(I), 0.);
      }
  }

  // adding some value function weights
  vector<double>& vfnweights = fval->getWeight();
  const auto w = cpputil::ordered_vec(vfnweights.size(),1);
  std::copy(w.begin(), w.end(), vfnweights.begin());
  ALPPlanner alp = boost::make_shared<_ALPPlanner>(fmdp, 0.9);
  alp->setFactoredValueFunction(fval);
  alp->precompute();

  // compute factored bellman functionals (1)
  FunctionSet<Reward> F = algorithm::factoredBellmanFunctionals(domain, fval);
  const SizeVec elim_order_s = get_state_vars(domain, {});
  auto tplBe = algorithm::variableElimination(F, elim_order_s);
  double maxVal = 0.;
  EXPECT_TRUE(F.empty());
  for(const auto& empty_fn : std::get<1>(tplBe)) {
      maxVal += empty_fn->eval(State(),Action());
  }

  // compute factored bellman functionals (2)
  FunctionSet<Reward> F2 = algorithm::factoredBellmanFunctionals(domain, fval);
  SizeVec mutable_elim_s = elim_order_s;
  tplBe = algorithm::variableEliminationHeur(F2, mutable_elim_s);
  EXPECT_EQ(elim_order_s, mutable_elim_s);
  double maxVal2 = 0.;
  EXPECT_TRUE(F2.empty());
  for(const auto& empty_fn : std::get<1>(tplBe)) {
      maxVal2 += empty_fn->eval(State(),Action());
  }
  EXPECT_EQ(maxVal, maxVal2);

  // compute factored bellman functionals (3)
  FunctionSet<Reward> F3 = algorithm::factoredBellmanFunctionals(domain, fval);
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
