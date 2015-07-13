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
//  virtual ~FunctionSetTest() { }
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
  //f2.pack();
  //EXPECT_THROW(f += f2, cpputil::InvalidException); // f2 still contains action scope, is now allowed as per new implementation

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
  const DiscreteFunction<double>& rf0 = algorithm::instantiate(&f, _s, true);
  EXPECT_DOUBLE_EQ(rf0->eval(_a), 3.);

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

///
/// \brief Function max marginalization tests
///
TEST_F(FunctionSetTest, FunctionMaxMarginalTest) {
    _FDiscreteFunction<double> f(_domain);
    f.addStateFactor(0);
    f.addStateFactor(1);
    f.addActionFactor(0);
    f.pack();
    f.define(_s,_a, 3);
    f.define(_s2,_a, 4);
    // maximize over first state factor
    _FDiscreteFunction<double> f2(f);
    const DiscreteFunction<double>& rf = algorithm::maximize(&f2, 0, true);
    EXPECT_TRUE(rf->getSubdomain()->getNumStateFactors() == 1);
    EXPECT_DOUBLE_EQ(rf->eval(State(),Action()), 4.);

    // maximize over action factor
    const DiscreteFunction<double>& rf2 = algorithm::maximize(rf.get(), _domain->getNumStateFactors(), true);
    EXPECT_TRUE(rf2->getSubdomain()->getNumActionFactors() == 0);

    // maximize over remaining state factor
    const DiscreteFunction<double>& rf3 = algorithm::maximize(rf2.get(), 1, false); // different code path
    EXPECT_TRUE(rf3->getSubdomain()->getNumStateFactors() == 0);
    EXPECT_DOUBLE_EQ(rf3->eval(State(),Action()), 4.);
#if !NDEBUG
    LOG_DEBUG("Final result after max marginalization:");
    _StateActionIncrementIterator saitr(rf3->getSubdomain());
    while(saitr.hasNext()) {
        const std::tuple<State,Action>& sa = saitr.next();
        const State& s = std::get<0>(sa);
        const Action& a = std::get<1>(sa);
        LOG_DEBUG(s << " " << a << ": " << rf3->eval(s,a));
    }
#endif
    // test with non-flat function
    Indicator<> I = boost::make_shared<_Indicator<>>(_domain, SizeVec({1}), _s2);
    const DiscreteFunction<double>& rf4 = algorithm::maximize(I.get(), 1, false);
    EXPECT_DOUBLE_EQ(rf4->eval(State(),Action()), 1.);
}

///
/// \brief Function join tests
///
TEST_F(FunctionSetTest, FunctionJoinTest) {
    FDiscreteFunction<double> f = boost::make_shared<_FDiscreteFunction<double>>(_domain);
    f->addStateFactor(0);
    f->addActionFactor(0);
    FDiscreteFunction<double> f2 = boost::make_shared<_FDiscreteFunction<double>>(*f);
    f2->addStateFactor(1);
    f->pack();
    f2->pack();
    f->define(_s,_a, 3);
    f->define(_s2,_a, 4);
    f2->define(_s,_a, 10);
    f2->define(_s2,_a, 20);

    const DiscreteFunction<double>& res = algorithm::join<double>({f,f2});
    EXPECT_TRUE(res->getSubdomain()->getNumStateFactors() == 2 && res->getSubdomain()->getNumActionFactors() == 1);
    EXPECT_DOUBLE_EQ(res->eval(_s,_a), 13.);
}

///
/// \brief Function slicing tests
///
TEST_F(FunctionSetTest, FunctionSliceTest) {
    FDiscreteFunction<double> f = boost::make_shared<_FDiscreteFunction<double>>(_domain);
    f->addStateFactor(0);
    f->addStateFactor(1);
    f->addActionFactor(0);
    f->pack();
    f->define(_s,_a, 3);
    f->define(_s2,_a, 4);
#if !NDEBUG
    LOG_DEBUG("Before slicing:");
    _StateActionIncrementIterator saitr(f->getSubdomain());
    while(saitr.hasNext()) {
        const std::tuple<State,Action>& sa = saitr.next();
        const State& s = std::get<0>(sa);
        const Action& a = std::get<1>(sa);
        LOG_DEBUG(s << " " << a << ": " << f->eval(s,a));
    }
#endif

    // slice of the first state vector
    std::vector<double> res = algorithm::slice(f.get(), 0, _s, _a);
    EXPECT_DOUBLE_EQ(res[_s.getFactor(0)], f->eval(_s,_a));
    EXPECT_DOUBLE_EQ(res[_s2.getFactor(0)], f->eval(_s2,_a));
#if !NDEBUG
    LOG_DEBUG("First slicing result:");
    for(auto v : res) {
        LOG_DEBUG(v);
    }
#endif
    // get a different slice
    State sl(_s);
    sl.setFactor(1,1);
    res = algorithm::slice(f.get(), 0, sl, _a);
    EXPECT_DOUBLE_EQ(res[sl.getFactor(0)], f->eval(sl,_a));
#if !NDEBUG
    LOG_DEBUG("Second slicing result:");
    for(auto v : res) {
        LOG_DEBUG(v);
    }
#endif
}

///
/// \brief Lifted operations (#) tests
///
TEST_F(FunctionSetTest, LiftedOperationTest) {
  FDiscreteFunction<double> f = boost::make_shared<_FDiscreteFunction<double>>(_domain);
  f->addStateFactor(0);
  f->addLiftedFactor(boost::make_shared<_LiftedFactor>(std::initializer_list<Size>{1,0}));
  f->pack();
  // check allocated size for generated subdomain
  EXPECT_EQ(f->values().size(), 4 * 3);

  boost::shared_ptr<FunctionSet<double>> f_set = boost::make_shared<FunctionSet<double>>(_domain);
  f_set->insert(f);
  EXPECT_EQ(f_set->getFunctions().size(), 1);
  EXPECT_EQ(f_set->getNumFactors(), 2); // includes lifted operation

  LOG_DEBUG("Function set:");
  LOG_DEBUG(*f_set);

  f_set->eraseStateFactor(1);
  EXPECT_TRUE(f_set->empty());
}

///
/// \brief Simple compression of lifted function test
///
TEST_F(FunctionSetTest, LiftedCompressionTest) {
  typedef std::unordered_multimap<std::size_t,Size> HashSizeMap;

  // create a new domain with 100 state factors
  _domain = boost::make_shared<_Domain>();
  for(int i = 0; i < 100; i++) {
    _domain->addStateFactor(0, 1);
  }
  _domain->setRewardRange(-1, 0);

  _EmptyFunction<Reward> E(_domain);
  SizeVec proper{ 0, 3, 5, 13, 20, 22, 23,27 };
  for(Size v : proper) {
    E.addStateFactor(v);
  }

  std::vector<SizeVec> liftedVars{ {6,8,25}, {4,17,24}, {3,5,13,20}, {4,14,18,25}, {0,7,15,24}, {6,18,23,29}, {6,16,19,20}, {6,14,18,25}, {9,19,21,27}, {0,5,22,23,27} };
  for(const auto& lvec : liftedVars) {
      LiftedFactor lf = boost::make_shared<_LiftedFactor>(lvec);
      E.addLiftedFactor(std::move(lf));
  }
  LOG_INFO("E before compression: " << E);
  LOG_INFO("E state/lifted factors before compression: " << E.getStateFactors().size() << " " << E.getLiftedFactors().size());
  E.computeSubdomain();
  Size num_states = E.getSubdomain()->getNumStates();
  LOG_INFO("E subdomain joint states before compression: " << num_states);
  EXPECT_EQ(num_states, 1920000000);

  HashSizeMap hsm; // store removed variables
  auto liftVec = E.compress(&hsm);
  LOG_INFO("E after compression: " << E);
  LOG_INFO("E state/lifted factors after compression: " << E.getStateFactors().size() << " " << E.getLiftedFactors().size());
  E.computeSubdomain();
  Size num_states_c = E.getSubdomain()->getNumStates();
  LOG_INFO("E subdomain joint states after compression: " << num_states_c);
  EXPECT_EQ(num_states_c, 4718592);
  LOG_INFO("Compression ratio: " << static_cast<double>(num_states_c)/num_states);

  // test that removed variables are recorded correctly in `hsm'
  const LiftedVec& lvec_c = E.getLiftedFactors(); // lifted factors in compressed domain
  LiftedFactor lf = boost::make_shared<_LiftedFactor>(std::initializer_list<Size>{}); // dummy factor for search
  auto lifted_comp = [](const LiftedFactor& l, const LiftedFactor& o) -> bool { return *l < *o; }; // comparison function
  for(const auto& p : liftVec) { // over all modified lifted factors
    const std::size_t o_hash = p.first;
    const std::size_t n_hash = p.second;
    SizeVec o_state_factors; // reconstruction of original state factors
    if(n_hash != _LiftedFactor::EMPTY_HASH) { // look up remaining factors in compressed domain
      lf->setHash(n_hash);
      auto it = std::lower_bound(lvec_c.begin(), lvec_c.end(), lf, lifted_comp);
      ASSERT_TRUE(it != lvec_c.end() && *(*it) == *lf);
      // add removed variables back to lifted factor and compare hashes
      o_state_factors = (*it)->getStateFactors(); // reduced state factor set in compressed domain
    }
    const auto range = hsm.equal_range(o_hash); // obtain removed state variables in `hsm' mapping
    const auto end = range.second;
    for (auto hsmIt = range.first; hsmIt != end; ++hsmIt) {
      o_state_factors.push_back(hsmIt->second);
    }
    std::sort(o_state_factors.begin(), o_state_factors.end());
    EXPECT_EQ(boost::hash_range(o_state_factors.begin(), o_state_factors.end()), o_hash);
  }
#if !NDEBUG
  // loop over removed variables
  for(auto hsmIt = hsm.begin(); hsmIt != hsm.end(); ) {
      const Size hash = hsmIt->first; // lifted counter under consideration
      const auto range = hsm.equal_range(hash);
      const auto end = range.second;
      LOG_DEBUG(hash << ": ");
      for (; hsmIt != end; ++hsmIt) {
        LOG_DEBUG(hsmIt->second);
      }
  }
#endif

  // Test another, simpler function
  _EmptyFunction<Reward> E2(_domain);
  E2.addStateFactor(0);
  lf = boost::make_shared<_LiftedFactor>(std::initializer_list<Size>{0});
  E2.addLiftedFactor(std::move(lf));
  LOG_INFO("E2 before compression: " << E2);
  LOG_INFO("E2 state/lifted factors before compression: " << E2.getStateFactors().size() << " " << E2.getLiftedFactors().size());
  E2.computeSubdomain();
  num_states = E2.getSubdomain()->getNumStates();
  LOG_INFO("E2 subdomain joint states before compression: " << num_states);
  EXPECT_EQ(num_states, 4);

  hsm.clear();
  E2.compress(&hsm);
  LOG_INFO("E2 after compression: " << E2);
  LOG_INFO("E2 state/lifted factors after compression: " << E2.getStateFactors().size() << " " << E2.getLiftedFactors().size());
  E2.computeSubdomain();
  num_states_c = E2.getSubdomain()->getNumStates();
  LOG_INFO("E2 subdomain joint states after compression: " << num_states_c);
  EXPECT_EQ(num_states_c, 2);
  LOG_INFO("Compression ratio: " << static_cast<double>(num_states_c)/num_states);

  EXPECT_EQ(hsm.size(),1);
  auto hsmIt = hsm.begin();
  EXPECT_EQ(hsmIt->second, 0);
}
