/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <gtest/gtest.h>
#include <fstream>

#include "crl/alp.hpp"
#include "crl/basis_gen.hpp"
#include "crl/env_sysadmin.hpp"
#include "crl/vi.hpp"
#include "crl/conversions.hpp"
#include "logger.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

//
// More complex integration test that runs the ALP solver on the SysAdmin environment
//

namespace {

///
/// \brief Run value-iteration on flat MDP
///
QTable testVI(MDP mdp, Domain domain, float gamma) {
    long start_time = time_in_milli();
    VIPlanner planner(new _FlatVIPlanner(domain, mdp, .0001, gamma));
    planner->plan();
    long end_time = time_in_milli();
    LOG_INFO("VI planner returned after " << end_time - start_time << "ms");
    QTable qtable = planner->getQTable();
    return qtable;
}

} // anonymous ns

///
/// \brief Compare ALP results for an exhaustive indicator basis with VI value function
///
TEST(ALPIntegrationTest, TestSysadminExhaustiveBasis) {
  srand(time(NULL));

  sysadmin::Sysadmin thesys = buildSysadmin("ring", 1);
  Domain domain = thesys->getDomain();
  FactoredMDP fmdp = thesys->getFactoredMDP();
  LOG_INFO(fmdp->T());

  FactoredValueFunction fval = boost::make_shared<_FactoredValueFunction>(domain);

  // insert exhaustive indicator basis
  _StateIncrementIterator sitr(domain);
  while(sitr.hasNext()) {
      auto I = boost::make_shared<_Indicator<Reward>>(domain, cpputil::ordered_vec<Size>(domain->getNumStateFactors()), sitr.next());
      fval->addBasisFunction(std::move(I), 0.);
  }

  // run the ALP planner
  _ALPPlanner planner(fmdp, 0.9);

  long start_time = time_in_milli();
  planner.setFactoredValueFunction(fval); // this will be computed
  int res = planner.plan();
  long end_time = time_in_milli();
  LOG_INFO("FALP planner returned after " << end_time - start_time << "ms");

  EXPECT_EQ(res, 0) << "ALP " << (res == 1 ? "generateLP()" : "solve()") << " failed"; // else: lp successfully generated

  if(!res) { // success
    LOG_INFO("Results:");
    for(auto w : fval->getWeight()) {
        LOG_INFO(" W: " << w);
    }

    //
    // Compare against value iteration solution
    //
    if(fval->getBasis().size() == domain->getNumStates()) {
        LOG_INFO("Comparing with flat VI solution:");

        MDP mdp = convertToMDP(fmdp);
        QTable qt = testVI(mdp, domain, 0.9);
        const vector<double>& wvec = fval->getWeight();
        vector<double>::size_type si = 0;

        _StateIncrementIterator sitr(domain);
        while(sitr.hasNext()) {
          const State& s = sitr.next();
          double r = qt->getV(s);

          EXPECT_TRUE(cpputil::approxEq(r, wvec[si], 0.1)); // comparison across algorithms, loose accuracy constraint
          EXPECT_NEAR(wvec[si++], fval->getV(s), 2*std::numeric_limits<Reward>::epsilon()); // tighter constraint
          LOG_INFO(" V_vi(" << s << ")=" << qt->getV(s) << " -- V_alp=" << (double)fval->getV(s));
        }
    }

    //
    // Compute maximum value for a specific state
    //
    State js(domain);
    js.setFactor(0,2);
    js.setFactor(1,2);
    // eliminate only action variables, in order
    const SizeVec elim_order = cpputil::ordered_vec<Size>(domain->getNumActionFactors(), domain->getNumStateFactors());
    tuple<Action,Reward> res = fval->getBestAction(js, elim_order);
    LOG_INFO("Maximum value in " << js << " after variable elimination: " << std::get<1>(res));
    EXPECT_DOUBLE_EQ(std::get<1>(res), fval->getQ(js,std::get<0>(res)));
    EXPECT_TRUE(fval->getV(js) >= fval->getQ(js,std::get<0>(res))); // in all approximations, V should be an upper bound on Q*
    LOG_INFO("Maximizing action is " << std::get<0>(res));
    // test against some other action
    Action oa(std::get<0>(res));
    oa.setIndex(0);
    LOG_DEBUG("Q value in " << js << " and another action " << oa << ": " << fval->getQ(js,oa));
    EXPECT_TRUE(fval->getV(js) >= fval->getQ(js,oa));

    //
    // Obtain (factored) max-Q function
    //
//  FunctionSet<Reward> f_set = fval->getMaxQ(elim_order);
    FunctionSet<Reward> f_set = fval->getMaxQ();
    auto qfns = f_set.getFunctions();
    LOG_INFO("MaxQ-Function has " << qfns.size() << " local terms: ");
    for(auto& qf : qfns) {
        LOG_INFO("{ " << qf->getSubdomain()->getNumStateFactors() << " }");
    }

    //
    // Compare value function and max-Q function
    //
    _StateIncrementIterator sitr(domain);
    while(sitr.hasNext()) {
      const State& s = sitr.next();
      double ret = 0.;
      for(auto& qf : qfns) {
          State ms = qf->mapState(s);
          ret += qf->eval(ms,Action());
      }
      double v = fval->getV(s);
      LOG_INFO("V_alp=(" << s << ")=" << v << " -- maxQ=" << ret << " -- delta=" << v-ret);
      // small padding to counter numeric issues
      EXPECT_TRUE(v + 1e-10 >= ret);
    }
  }
}

///
/// \brief Tests conjunctive basis construction
///
TEST(ALPIntegrationTest, TestConjunctiveBasis) {
  srand(time(NULL));

  sysadmin::Sysadmin thesys = buildSimpleSysadmin("ring", 4);
  Domain domain = thesys->getDomain();
  FactoredMDP fmdp = thesys->getFactoredMDP();
  const RangeVec& ranges = domain->getStateRanges();
  LOG_INFO(fmdp->T());

  // factored value function with single factor basis functions
  FactoredValueFunction fval = boost::make_shared<_FactoredValueFunction>(domain);
  for(Size fa = 0; fa < ranges.size(); fa++) { // assumption: DBN covers all domain variables
      auto I_o = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa}), State(domain,0));
      _StateIncrementIterator sitr(I_o->getSubdomain());
      while(sitr.hasNext()) {
          auto I = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa}), sitr.next());
          fval->addBasisFunction(std::move(I), 0.);
      }
  }

  // run the ALP planner
  _ALPPlanner planner(fmdp, 0.9);
  planner.setFactoredValueFunction(fval); // this will be computed
  int res = planner.plan();
  EXPECT_EQ(res, 0) << "ALP " << (res == 1 ? "generateLP()" : "solve()") << " failed";

  // maxQ-function analysis (keep track of `action coverage')
  auto maxQ = fval->getMaxQ<_FConjunctiveFeature<Reward>>().getFunctions();
  EXPECT_EQ(maxQ.size(), 4); // corresponding to scopes of all backprojections

  // maxQ-function eliminates all action factors (and returns _FConjunctiveFeature for modified terms)
  Size count_fcj = 0, count_fd = 0;
  for(const auto& fn : maxQ) {
      const Conjunction* cf = dynamic_cast<const Conjunction*>(fn.get());
      if(cf)
          count_fcj++;
      else
          count_fd++;
  }
  EXPECT_EQ(count_fd, 0); // all basis pairs (with a reward function) share scope and are joined in maxQ-function
  EXPECT_EQ(count_fcj, fval->getBasis().size()/2);

  // Bellman residual should maintain that number of _FConjunctiveFeature and also include basis functions
  auto bellres = algorithm::factoredBellmanFunctionals<_FConjunctiveFeature<Reward>>(domain, fval).getFunctions();
  Size count_fcj2 = 0;
  for(const auto& fn : bellres) {
      const Conjunction* cf = dynamic_cast<const Conjunction*>(fn.get());
      if(cf) {
          count_fcj2++;
          // test recorded `action-connectivity' (specific to sysadmin)
          const SizeVec& fadom = cf->getBaseFeatures();
          EXPECT_EQ(fadom.size(), 1);
          const SizeVec& sf = fn->getStateFactors();
          EXPECT_TRUE(cpputil::has_intersection(fadom.begin(), fadom.end(), sf.begin(), sf.end()));
      }
      else
          count_fd++;
  }
  EXPECT_EQ(count_fd, fval->getBasis().size()); // all basis pairs (with a reward function) share scope and are joined in maxQ-function
  EXPECT_EQ(count_fcj2, count_fcj);

  LOG_INFO("Computing next best basis to insert...");
  BinaryBasisGenerator<NChooseTwoIterator<Size,SizeVec>,OptBEBFScore> basisGen(domain, fval, fmdp, "bebf-test");
  DiscreteFunction<Reward> nextBasis = basisGen.nextBest();
  EXPECT_TRUE(nextBasis != nullptr) << "No next basis, nullptr returned.";
  LOG_INFO("Next best: " << (*nextBasis));

  // note: here the conjunction is over basis functions
  const Conjunction* cf = dynamic_cast<const Conjunction*>(nextBasis.get());
  EXPECT_TRUE(cf != nullptr);
  EXPECT_EQ(cf->getBaseFeatures().size(), 2);
  // problem specific test: basis connects start/end
  EXPECT_EQ(cf->getBaseFeatures()[0], 0);
  EXPECT_EQ(cf->getBaseFeatures()[1], 7);

  // insert feature into basis
  basisGen.addToBlacklist(*cf);
  fval->addBasisFunction(std::move(nextBasis), 0.);
  fval->clearBackprojections();

  // loop over new basis and make sure exactly one conjunctive feature exists over <h0,h7>
  Size count = 0;
  for(const auto& fn : fval->getBasis()) {
      const Conjunction* cf = dynamic_cast<const Conjunction*>(fn.get());
      if(cf) {
          count++;
          EXPECT_EQ(cf->getBaseFeatures()[0], 0);
          EXPECT_EQ(cf->getBaseFeatures()[1], 7);
          LOG_DEBUG("New basis is over original base features: " << *cf);
      }
  }
  EXPECT_EQ(count, 1);

  // test: this should recompute the backprojections but since nextBasis is still associated with 0, not change maxQ
  planner.precompute();
  auto maxQ2 = fval->getMaxQ<_FConjunctiveFeature<Reward>>().getFunctions();
  EXPECT_EQ(maxQ2.size(), maxQ.size());

  // run another iteration of planning, maxQ should then be different
  fval->clearBackprojections();
  int res2 = planner.plan();
  EXPECT_EQ(res2, 0) << "ALP " << (res2 == 1 ? "generateLP()" : "solve()") << " failed";
  auto maxQ3 = fval->getMaxQ<_FConjunctiveFeature<Reward>>().getFunctions();
  EXPECT_EQ(maxQ3.size(), maxQ2.size()-1);

  // solution associates w value with new conjunctive basis function
  EXPECT_NE(fval->getWeight()[fval->getBasis().size()-1], 0.);
  // test `action-connectivity'
  Size maxASc = 0;
  Size SSc = 0;
  for(const auto& fn : maxQ3) {
      const Conjunction* cf = dynamic_cast<const Conjunction*>(fn.get());
      ASSERT_TRUE(cf != nullptr); // problem specific since lrfs include action factors
      SizeVec::size_type si = cf->getBaseFeatures().size();
      if(si > maxASc) {
          maxASc = si;
          SSc = fn->getStateFactors().size();
          LOG_DEBUG("maxQ factor: " << *fn << " couples actions: " << *cf);
      }
  }
  EXPECT_EQ(maxASc, 2);
  EXPECT_EQ(SSc, 3);

  // insert another conjunctive basis function manually
  Size h1_id = 4;
  Size h2_id = 6;
  DiscreteFunction<Reward> candf = algorithm::binpair(h1_id,h2_id,fval->getBasis()[h1_id],fval->getBasis()[h2_id]);
  ASSERT_TRUE(candf != nullptr);
  fval->addBasisFunction(std::move(candf), 0.);
  // no blacklisting tested here
  fval->clearBackprojections();
  int res3 = planner.plan();
  EXPECT_EQ(res3, 0) << "ALP " << (res2 == 1 ? "generateLP()" : "solve()") << " failed";
  // solution associates w value with new conjunctive basis function
  EXPECT_NE(fval->getWeight()[fval->getBasis().size()-1], 0.);
  auto maxQ4 = fval->getMaxQ<_FConjunctiveFeature<Reward>>().getFunctions();

  // solution associates w value with new conjunctive basis function
  EXPECT_EQ(maxQ4.size(), maxQ3.size()-1);
  // test `action-connectivity'
  Size maxASc2 = 0;
  Size SSc2 = 0;
  for(const auto& fn : maxQ4) {
      const Conjunction* cf = dynamic_cast<const Conjunction*>(fn.get());
      ASSERT_TRUE(cf != nullptr); // problem specific since lrfs include action factors
      SizeVec::size_type si = cf->getBaseFeatures().size();
      if(si > maxASc2) {
          maxASc2 = si;
          SSc2 = fn->getStateFactors().size();
          LOG_DEBUG("maxQ factor (manual insertion): " << *fn << " couples actions: " << *cf);
      }
  }
  EXPECT_EQ(maxASc2, 3);
  EXPECT_EQ(SSc2, 4);
}


///
/// \brief More involved test that also does factored Bellman computations in larger SysAdmin
///
TEST(ALPIntegrationTest, TestFactoredBellmanResiduals) {
  srand(time(NULL));

  sysadmin::Sysadmin thesys = buildSysadmin("ring", 4);
  Domain domain = thesys->getDomain();
  FactoredMDP fmdp = thesys->getFactoredMDP();
  LOG_INFO(fmdp->T());

#if 0
  // debug print the CPTs
  for(Size i = 0; i < fmdp->T().size(); i++) {
      const auto& fa = fmdp->T().factor(i);
      LOG_DEBUG(*fa << " CPT: ");
      _StateActionIncrementIterator saitr(fa->getSubdomain());
      while(saitr.hasNext()) {
          const std::tuple<State,Action>& sa = saitr.next();
          const State& s = std::get<0>(sa);
          const Action& a = std::get<1>(sa);
          LOG_DEBUG(" " << s << " " << a << ": " << (*fa)(s,a));
      }
  }
#endif

  FactoredValueFunction fval = boost::make_shared<_FactoredValueFunction>(domain);
  // add the constant basis (to guarantee LP feasibility)
//  auto cfn = boost::make_shared<_ConstantFn<Reward>>(domain);
//  fval->addBasisFunction(std::move(cfn), 0.);
  // add more basis functions
  const RangeVec& ranges = domain->getStateRanges();
#if 0
  // create a basis (one indicator per local state, i.e., corresponding to `single' basis in Guestrin thesis)
  for(Size fa = 0; fa < ranges.size(); fa++) { // assumption: DBN covers all domain variables
      // place one indicator basis on each possible factor value
      for(Factor fv=0; fv<=ranges[fa].getSpan(); fv++) {
          State dummy_s; // we don't care about actual domain here
          dummy_s.setIndex(ranges[fa].getMin()+fv);
          auto I = boost::make_shared<_Indicator<Reward>>(domain);
          I->addStateFactor(fa);
          I->setState(dummy_s);
          fval->addBasisFunction(std::move(I), 0.);
      }
  }
#endif
#if 0
  for(Size fa = 0; fa < ranges.size(); fa+=2) { // over all status variables
    auto I = boost::make_shared<_Indicator<Reward>>(domain);
    I->addStateFactor(fa);
    State dummy_s; // we don't care about actual domain here
    dummy_s.setIndex((Factor)sysadmin::Status::GOOD);
    I->setState(dummy_s);
    fval->addBasisFunction(std::move(I), 0.);
  }
#endif
#if 0
  // exhaustive indicator basis
  _StateIncrementIterator sitr(domain);
  while(sitr.hasNext()) {
      auto I = boost::make_shared<_Indicator<Reward>>(domain, cpputil::ordered_vec<Size>(domain->getNumStateFactors()), sitr.next());
      fval->addBasisFunction(std::move(I), 0.);
  }
#endif

  for(Size fa = 0; fa < ranges.size(); fa++) { // assumption: DBN covers all domain variables
      auto I_o = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa}), State(domain,0));
      _StateIncrementIterator sitr(I_o->getSubdomain());
      while(sitr.hasNext()) {
          auto I = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa}), sitr.next());
          fval->addBasisFunction(std::move(I), 0.);
      }
  }
#if 0
  Size h1_id = 22;
  Size h2_id = 44;
  DiscreteFunction<Reward> candf = algorithm::binpair(h1_id,h2_id,fval->getBasis()[h1_id],fval->getBasis()[h2_id]);
  assert(candf);
  fval->addBasisFunction(std::move(candf), 0.);
#endif
#if 0
  for(Size fa = 0; fa < ranges.size(); fa+=2) { // assumption: DBN covers all domain variables
      auto I_o = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa,fa+1}), State(domain,0));
      _StateIncrementIterator sitr(I_o->getSubdomain());
      while(sitr.hasNext()) {
          auto I = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa,fa+1}), sitr.next());
          fval->addBasisFunction(std::move(I), 0.);
      }
  }

  // add a link
  auto I_o = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({1,2}), State(domain,0));
  _StateIncrementIterator sitr(I_o->getSubdomain());
  while(sitr.hasNext()) {
      auto I = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({1,2}), sitr.next());
      fval->addBasisFunction(std::move(I), 0.);
  }
#endif
#if 0
  // add some links between agents
  for(Size fa = 1; fa < ranges.size(); fa+=2) { // assumption: DBN covers all domain variables
      if(fa+1<ranges.size()) {
        auto I_o = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa,fa+1}), State(domain,0));
        _StateIncrementIterator sitr(I_o->getSubdomain());
        while(sitr.hasNext()) {
            auto I = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa,fa+1}), sitr.next());
            fval->addBasisFunction(std::move(I), 0.);
        }
      }
  }
#endif

  // run the ALP planner
  _ALPPlanner planner(fmdp, 0.9);

  long start_time = time_in_milli();
  planner.setFactoredValueFunction(fval); // this will be computed
  int res = planner.plan();
  long end_time = time_in_milli();
  LOG_INFO("FALP planner returned after " << end_time - start_time << "ms");

  EXPECT_EQ(res, 0) << "ALP " << (res == 1 ? "generateLP()" : "solve()") << " failed"; // else: lp successfully generated

  if(!res) { // success
    LOG_INFO("Results:");
    for(auto w : fval->getWeight()) {
        LOG_INFO(" W: " << w);
    }
#if !NDEBUG
    LOG_DEBUG("Writing basis function weights to text file");
    {
      ofstream file("weights0.txt");
      for(double w : fval->getWeight()) {
        file << std::fixed << std::setprecision(std::numeric_limits<double>::digits10+2) << w << '\n';
      }
    }
#endif

    //
    // Compare against value iteration solution if we have an exact LP solution (exhaustive basis function set)
    //
    if(fval->getBasis().size() == domain->getNumStates()) {
        LOG_INFO("Comparing with flat VI solution:");

        MDP mdp = convertToMDP(fmdp);
        QTable qt = testVI(mdp, domain, 0.9);
        const vector<double>& wvec = fval->getWeight();
        vector<double>::size_type si = 0;

        _StateIncrementIterator sitr(domain);
        while(sitr.hasNext()) {
          const State& s = sitr.next();
          double r = qt->getV(s);

          EXPECT_TRUE(cpputil::approxEq(r, wvec[si], 0.1)); // comparison across algorithms, loose accuracy constraint
          EXPECT_NEAR(wvec[si++], fval->getV(s), 2*std::numeric_limits<Reward>::epsilon()); // tighter constraint
          LOG_INFO(" V_vi(" << s << ")=" << qt->getV(s) << " -- V_alp=" << (double)fval->getV(s));
        }
    }

    //
    // Compute maximum value for a specific state
    //
    State js(domain);
    js.setFactor(0,2);
    js.setFactor(1,2);
    // eliminate only action variables, in order
    const SizeVec elim_order_a = cpputil::ordered_vec<Size>(domain->getNumActionFactors(), domain->getNumStateFactors());
    tuple<Action,Reward> res = fval->getBestAction(js, elim_order_a);
    LOG_INFO("Maximum value in " << js << " after variable elimination: " << std::get<1>(res));
    EXPECT_DOUBLE_EQ(std::get<1>(res), fval->getQ(js,std::get<0>(res)));
    EXPECT_TRUE(fval->getV(js) >= fval->getQ(js,std::get<0>(res))); // in all approximations, V should be an upper bound on Q*
    LOG_INFO("Maximizing action is " << std::get<0>(res));
    // test against some other action
    Action oa(std::get<0>(res));
    oa.setIndex(0);
    LOG_DEBUG("Q value in " << js << " and another action " << oa << ": " << fval->getQ(js,oa));
    EXPECT_TRUE(fval->getV(js) >= fval->getQ(js,oa));

    //
    // Debug print max-Q function factorization
    //
//  FunctionSet<Reward> f_set = fval->getMaxQ(elim_order_a);
    FunctionSet<Reward> f_set = fval->getMaxQ();
    auto qfns = f_set.getFunctions();
    LOG_INFO("MaxQ-Function has " << qfns.size() << " local terms: ");
    for(auto& qf : qfns) {
        LOG_INFO("{ " << qf->getSubdomain()->getNumStateFactors() << " }");
    }

    //
    // Compute Bellman error
    //

    // First method
    const SizeVec elim_order_s = cpputil::ordered_vec<Size>(domain->getNumStateFactors());
    LOG_DEBUG("Running factoredBellmanError");
    double beval = algorithm::factoredBellmanError(domain, fval, elim_order_s);
    LOG_INFO("Bellman error (1): " << beval);

    // Second method: retain variable `1' in domain and maximize manually
    const SizeVec red_elim_order_s = get_state_vars(domain, {1});
    FunctionSet<Reward> F = algorithm::factoredBellmanFunctionals(domain, fval);
    auto tplBe = algorithm::variableElimination(F, red_elim_order_s, algorithm::maximize);
    double beval2 = 0.;
    // loop over empty functions
    for(const auto& ef : std::get<1>(tplBe)) {
        beval2 += ef->eval(State(),Action());
    }
    // maximize manually
    // loop over partially instantiated ones (dependence on `1')
    double maxVal = -std::numeric_limits<double>::infinity();
    for(Factor fa = 0; fa < 3; fa++) {
        double v = 0;
        const State dummy_s(domain, fa);
        for(const auto& f : F.getFunctions()) {
            v += f->eval(dummy_s,Action());
        }
        if(v > maxVal) {
            maxVal = v;
        }
    }
    beval2 += maxVal;
    EXPECT_NEAR(beval, beval2, 1e-14);
    LOG_DEBUG("Bellman error (2): " << beval2);

    // Third method: retain variable `1' in domain and maximize using evalOpBasis
    // maximize using evalOpBasis for a basis that is everywhere enabled in `1'
    _FDiscreteFunction<Reward> basis(domain);
    basis.addStateFactor(1);
    basis.pack(1); // set everywhere to enabled
    std::get<0>(tplBe) = std::move(F.getFunctions()); // make tplBe a valid `FactoredFunction'
    beval2 = algorithm::evalOpOverBasis(&basis, tplBe, true, -std::numeric_limits<double>::infinity(),
                                        [](Reward& v1, Reward& v2) { if(v2 > v1) { v1 = v2; } });
    EXPECT_NEAR(beval, beval2, 1e-14);

    // now for three separate indicator basis functions
    maxVal = -std::numeric_limits<double>::infinity();
    for(Factor fa = 0; fa < 3; fa++) {
        _Indicator<Reward> I(domain);
        I.addStateFactor(1);
        const State dummy_s(domain,fa);
        I.setState(dummy_s);
        double v = algorithm::evalOpOverBasis(&I, tplBe, false, -std::numeric_limits<double>::infinity(),
                                              [](Reward& v1, Reward& v2) { if(v2 > v1) { v1 = v2; } });
        LOG_DEBUG("Indicator on `Factor_1 == " << fa << " covers max BE of " << v);
        if(v > maxVal) {
            LOG_DEBUG("Indicator on `Factor_1 == " << fa << " (currently) holds max BE.");
            maxVal = v;
        }
    }
    EXPECT_NEAR(maxVal, beval2, 1e-14);
    LOG_DEBUG("Bellman error (3): " << beval2);

    // Forth method: retain variable `1' in domain and run variable elimination a second time
    beval2 = 0.;
    // loop over empty functions
    for(const auto& ef : std::get<1>(tplBe)) {
        beval2 += ef->eval(State(),Action());
    }
    // run variable elimination again over partially instantiated ones (dependence on `1')
    auto tplBe2 = algorithm::variableElimination(F, {1}, algorithm::maximize);
    // loop over empty functions
    for(const auto& ef : std::get<1>(tplBe2)) {
        beval2 += ef->eval(State(),Action());
    }
    EXPECT_NEAR(beval, beval2, 1e-14);
    LOG_DEBUG("Bellman error (4): " << beval2);

    //
    // Compute minimal Bellman residual
    //
    auto retFns = algorithm::factoredBellmanResidual(domain, fval, elim_order_s, algorithm::minimize);
    double minVal = 0.;
    for(const auto& empty_fn : std::get<1>(retFns)) {
        minVal += empty_fn->eval(State(),Action());
    }
    LOG_DEBUG("minVal: " << minVal);
    EXPECT_NEAR(minVal, 0., 1e-14);

    //
    // Compute Bellman residual `integral' over entire state space
    //

    // First method
    auto tplFn = algorithm::factoredBellmanMarginal(domain,{},fval);
    EXPECT_TRUE(std::get<0>(tplFn).empty());
    double intr1 = 0.;
    for(const auto& ef : std::get<1>(tplFn)) {
        intr1 += ef->eval(State(),Action());
    }
    LOG_INFO("Bellman residual integral: " << intr1);

    // Second method: retain variable `1' in domain and sum manually
    const SizeVec margOver = { 1 };
    _Indicator<> dummy_f(domain, margOver, State(domain,0)); // only used for domain computations
    auto partMarg = algorithm::factoredBellmanMarginal(domain,dummy_f.getStateFactors(),fval);
    EXPECT_FALSE(std::get<0>(partMarg).empty());
    double empty_intr = 0., intr2 = 0.;
    // loop over empty functions
    for(const auto& ef : std::get<1>(partMarg)) {
        empty_intr += ef->eval(State(),Action());
    }
    // loop over partially instantiated ones (dependence on `1')
    const subdom_map s_dom(dummy_f.getStateFactors());
    _StateIncrementIterator sitr(dummy_f.getSubdomain());
    while(sitr.hasNext()) {
        const State& s = sitr.next();
        intr2 += empty_intr;
        for(const auto& f : std::get<0>(partMarg)) {
            State ms = f->mapState(s,s_dom);
            intr2 += f->eval(ms,Action());
        }
    }
    EXPECT_NEAR(intr1, intr2, 1e-5);

    //
    // Compare value function and max-Q function
    //
    _StateIncrementIterator sitr2(domain);
    while(sitr2.hasNext()) {
      const State& s = sitr2.next();
      double ret = 0.;
      for(auto& qf : qfns) {
          State ms = qf->mapState(s);
          ret += qf->eval(ms,Action());
      }
      double v = fval->getV(s);
      LOG_INFO("V_alp=(" << s << ")=" << v << " -- maxQ=" << ret << " -- delta=" << v-ret);
      // small padding to counter numeric issues
      EXPECT_TRUE(v + 1e-10 >= ret);
    }

    //
    // Insert and resolve with next best functions twice (BEBFScore criterion)
    //

    for(int k = 0; k < 2; k++) {
        BinaryBasisGenerator<NChooseTwoIterator<Size,SizeVec>,BEBFScore> basisGen(domain, fval, fmdp, "bebf-test");
        DiscreteFunction<Reward> nextBasis = basisGen.nextBest();
        if(!nextBasis) {
          LOG_INFO("No next best basis: cost deemed infinite by Scoring function.");
          break;
        }

        // avoid double-insertion of same feature
        const Conjunction* cf = dynamic_cast<const Conjunction*>(nextBasis.get());
        assert(cf);
        basisGen.addToBlacklist(*cf);
        // insert feature into basis
        fval->clearBackprojections(); // TODO: implement SWAP so that recomputation of previous ones avoided
        fval->addBasisFunction(std::move(nextBasis), 0.);

        // run second iteration of planning
        long start_time = time_in_milli();
        // TODO: can init with old weights + 0 for valid solution..
        // TODO: don't have to deallocate lp every iteration
        int res2 = planner.plan();
        long end_time = time_in_milli();
        LOG_INFO("2nd: FALP planner returned after " << end_time - start_time << "ms");

        EXPECT_EQ(res2, 0) << "ALP " << (res2 == 1 ? "generateLP()" : "solve()") << " failed"; // else: lp successfully generated

        if(!res2) { // success
            LOG_INFO("2nd: Results:");
            for(auto w : fval->getWeight()) {
                LOG_INFO(" W: " << w);
            }
#if !NDEBUG
            LOG_DEBUG("Writing basis function weights to text file");
            {
                ofstream file("weights" + to_string(k+1) + ".txt");
                for(double w : fval->getWeight()) {
                    file << std::fixed << std::setprecision(std::numeric_limits<double>::digits10+2) << w << '\n';
                }
            }
            LOG_DEBUG("Writing conjunctive basis to text file");
            {
                ofstream file("conj" + to_string(k+1) + ".txt");
                for(const auto& fn : fval->getBasis()) {
                    const Conjunction* pc = dynamic_cast<const Conjunction*>(fn.get());
                    if(pc) {
                        const auto& baseFeatures = pc->getBaseFeatures();
                        for(int i = 0; i < baseFeatures.size(); i++) {
                            file << baseFeatures[i] << (i < baseFeatures.size()-1 ? "," : "");
                        }
                        file << '\n';
                    }
                }
            }
#endif
            LOG_DEBUG("2nd: Running factoredBellmanError");
            double beval = algorithm::factoredBellmanError(domain, fval, elim_order_s);
            LOG_INFO("2nd: Bellman error (1): " << beval);
        }
    }
  }
}
