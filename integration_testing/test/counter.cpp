/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <gtest/gtest.h>

#include "cpputil.hpp"
#include "crl/crl.hpp"
#include "crl/flat_tables.hpp"
#include "crl/lifted_ops.hpp"
#include "logger.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

namespace crl {

template<class T>
using FStateTable = boost::shared_ptr<_FStateTable<T>>;

}

namespace {

//
// Helper functions
//

/// \brief Remove state factor from given domain
/// \return Resulting \a Domain
/// \see maxStateFactor
Domain removeStateFactor(const Domain& d, std::string name) {
  Domain dn;
  dn = boost::make_shared<_Domain>();
  const RangeVec& d_ranges = d->getStateRanges();
  const StrVec& d_names = d->getStateNames();

  Size i = 0;
  for(const auto& n : d_names) {
    if(n != name) {
      dn->addStateFactor(d_ranges[i].getMin(), d_ranges[i].getMax(), n);
    }
    i++;
  }
  return dn;
}

/// \brief Remove binary state factor from given function and domain (max-marginal)
/// \return Resulting \a Domain and function
std::tuple<Domain,FStateTable<Reward>> maxStateFactor(const Domain& d, const FStateTable<Reward>& f, std::string name) {
  Domain dn = removeStateFactor(d, name);
  // offset of removed factor in original domain
  auto jitr = std::find(d->getStateNames().begin(), d->getStateNames().end(), name);
  assert(jitr != d->getStateNames().end());
  Size j = std::distance(d->getStateNames().begin(), jitr);

  FStateTable<Reward> fn = boost::make_shared<_FStateTable<Reward>>(dn);
  _StateIncrementIterator sitr(dn);
  while(sitr.hasNext()) {
      const State& s = sitr.next();
      State p1(d);
      Size offset = 0;
      for(Size i = 0; i < d->getNumStateFactors(); i++) {
        if(i == j) {
            p1.setFactor(i, 0);
            offset = 1;
            continue;
        }
        p1.setFactor(i, s.getFactor(i-offset));
      }

      // implement the max (binary state factor)
      State p2 = p1;
      p2.setFactor(j, 1);
      fn->setValue(s, std::max(f->getValue(p1), f->getValue(p2)));
  }

  return std::make_tuple(std::move(dn),std::move(fn));
}

/// \brief Reduce counter variable from a given domain by 1
/// \return Resulting \a Domain
/// \see maxCount
Domain removeCount(const Domain& d, std::string name) {
  Domain dn;
  dn = boost::make_shared<_Domain>();
  const RangeVec& d_ranges = d->getStateRanges();
  const StrVec& d_names = d->getStateNames();

  Size i = 0;
  for(const auto& n : d_names) {
      Factor fmin = d_ranges[i].getMin();
      Factor fmax = d_ranges[i].getMax();
      if(n == name) { // reduce count
          assert(fmax != 0);
          fmax--;
      }

      dn->addStateFactor(fmin, fmax, n);
      i++;
  }
  return dn;
}

/// \brief Remove a non-shared counter variable from given function and domain (max-marginal)
/// \return Resulting \a Domain and function
/// \note Non-shared counter is assumed, i.e., cannot appear in another counter's domain or as proper variable
std::tuple<Domain,FStateTable<Reward>> maxCount(const Domain& d, const FStateTable<Reward>& f, std::string name) {
  Domain dn = removeCount(d, name);
  const StrVec& dn_names = dn->getStateNames();

  FStateTable<Reward> fn = boost::make_shared<_FStateTable<Reward>>(dn);
  _StateIncrementIterator sitr(dn);
  while(sitr.hasNext()) {
      const State& s = sitr.next();
      State p1(d);
      Size i;
      for(i = 0; i < dn_names.size(); i++) {
        p1.setFactor(i, s.getFactor(i));
      }
      // implement the max
      State p2 = p1;
      i = 0;
      for(const auto& n : dn_names) {
          if(n == name) {
              p2.setFactor(i, s.getFactor(i)+1);
          }
          i++;
      }
      fn->setValue(s, std::max(f->getValue(p1), f->getValue(p2)));
  }

  return std::make_tuple(std::move(dn),std::move(fn));
}

} // anonymous ns

///
/// \brief Test variable elimination with proper, (non-shared) counter, and shared counter variables
///
TEST(CounterTest, BasicCounterTest) {
  srand(time(NULL));

  // f_LHS(A,B,#_A{A1,A2,A3,A4,A5},#_B{B1,B2,A3,A4})
  Domain d1;
  d1 = boost::make_shared<_Domain>();
  d1->addStateFactor(0, 1, "A"); // 2 states
  d1->addStateFactor(0, 1, "B"); // 2 states
  d1->addStateFactor(0, 5, "#_A"); // 6 states
  d1->addStateFactor(0, 4, "#_B"); // 5 states

  FStateTable<Reward> f_lhs = boost::make_shared<_FStateTable<Reward>>(d1);
  _StateIncrementIterator litr(d1);
  while(litr.hasNext()) {
    const State& s = litr.next();
    f_lhs->setValue(s, cpputil::randDouble());
  }

#if !NDEBUG
  LOG_DEBUG("f_lhs:");
  litr.reset();
  while(litr.hasNext()) {
    const State& s = litr.next();
    LOG_DEBUG(s << " " << f_lhs->getValue(s));
  }
#endif

  // f_RHS(A,B,#{A3,A4},#_A{A1,A2,A5},#_B{B1,B2})
  Domain d2;
  d2 = boost::make_shared<_Domain>();
  d2->addStateFactor(0, 1, "A"); // 2 states
  d2->addStateFactor(0, 1, "B"); // 2 states
  d2->addStateFactor(0, 2, "#"); // 3 states
  d2->addStateFactor(0, 3, "#_A"); // 4 states
  d2->addStateFactor(0, 2, "#_B"); // 3 states

  FStateTable<Reward> f_rhs = boost::make_shared<_FStateTable<Reward>>(d2);
  _StateIncrementIterator ritr(d2);
  while(ritr.hasNext()) {
      const State& s = ritr.next();
      // create corresponding state in d1
      State s1(d1);
      s1.setFactor(0, s.getFactor(0));
      s1.setFactor(1, s.getFactor(1));
      s1.setFactor(2, s.getFactor(2)+s.getFactor(3));
      s1.setFactor(3, s.getFactor(2)+s.getFactor(4));
      f_rhs->setValue(s, f_lhs->getValue(s1));
  }

#if !NDEBUG
  LOG_DEBUG("f_rhs:");
  ritr.reset();
  while(ritr.hasNext()) {
    const State& s = ritr.next();
    LOG_DEBUG(s << " " << f_rhs->getValue(s));
  }
#endif

  //
  // f_lhs non-shared
  //

  // eliminate A1 from f_lhs
  auto ltpl = maxCount(d1, f_lhs, "#_A");
  d1 = std::get<0>(ltpl);
  f_lhs = std::get<1>(ltpl);
  // eliminate A2 from f_lhs
  ltpl = maxCount(d1, f_lhs, "#_A");
  d1 = std::get<0>(ltpl);
  f_lhs = std::get<1>(ltpl);
  // eliminate A5 from f_lhs
  ltpl = maxCount(d1, f_lhs, "#_A");
  d1 = std::get<0>(ltpl);
  f_lhs = std::get<1>(ltpl);
  // eliminate B1 from f_lhs
  ltpl = maxCount(d1, f_lhs, "#_B");
  d1 = std::get<0>(ltpl);
  f_lhs = std::get<1>(ltpl);
  // eliminate B2 from f_lhs
  ltpl = maxCount(d1, f_lhs, "#_B");
  d1 = std::get<0>(ltpl);
  f_lhs = std::get<1>(ltpl);
#if !NDEBUG
  LOG_DEBUG("Intermediate result for f_lhs:");
  _StateIncrementIterator reslitr(d1);
  while(reslitr.hasNext()) {
      const State& s = reslitr.next();
      LOG_DEBUG(s << " " << f_lhs->getValue(s));
  }
#endif

  // manually eliminate shared count A3
  Domain dn = removeCount(d1, "#_A");
  dn = removeCount(dn, "#_B");

  FStateTable<Reward> fn = boost::make_shared<_FStateTable<Reward>>(dn);
  _StateIncrementIterator sitr(dn);
  while(sitr.hasNext()) {
      const State& s = sitr.next();
      State p1(d1);
      for(Size i = 0; i < dn->getNumStateFactors(); i++) {
        p1.setFactor(i, s.getFactor(i));
      }
      // implement the max
      State p2 = p1;
      p2.setFactor(2, s.getFactor(2)+1); // increment jointly (shared count)
      p2.setFactor(3, s.getFactor(3)+1);
      fn->setValue(s, std::max(f_lhs->getValue(p1), f_lhs->getValue(p2)));
  }
  // update domain, function
  d1 = dn;
  f_lhs = fn;

  // manually eliminate shared count A4
  dn = removeCount(d1, "#_A");
  dn = removeCount(dn, "#_B");

  fn = boost::make_shared<_FStateTable<Reward>>(dn);
  _StateIncrementIterator sitr2(dn);
  while(sitr2.hasNext()) {
      const State& s = sitr2.next();
      State p1(d1);
      for(Size i = 0; i < dn->getNumStateFactors(); i++) {
        p1.setFactor(i, s.getFactor(i));
      }
      // implement the max
      State p2 = p1;
      p2.setFactor(2, s.getFactor(2)+1); // increment jointly (shared count)
      p2.setFactor(3, s.getFactor(3)+1);
      fn->setValue(s, std::max(f_lhs->getValue(p1), f_lhs->getValue(p2)));
  }
  // update domain, function
  d1 = dn;
  f_lhs = fn;

  LOG_DEBUG("Removing state factors from f_lhs");
  ltpl = maxStateFactor(d1, f_lhs, "B");
  d1 = std::get<0>(ltpl);
  f_lhs = std::get<1>(ltpl);
  ltpl = maxStateFactor(d1, f_lhs, "A");
  d1 = std::get<0>(ltpl);
  f_lhs = std::get<1>(ltpl);

  //
  // f_rhs, all non-shared
  //

  auto rtpl = maxCount(d2, f_rhs, "#_A");
  d2 = std::get<0>(rtpl);
  f_rhs = std::get<1>(rtpl);
  rtpl = maxCount(d2, f_rhs, "#_A");
  d2 = std::get<0>(rtpl);
  f_rhs = std::get<1>(rtpl);
  rtpl = maxCount(d2, f_rhs, "#_A");
  d2 = std::get<0>(rtpl);
  f_rhs = std::get<1>(rtpl);
  rtpl = maxCount(d2, f_rhs, "#_B");
  d2 = std::get<0>(rtpl);
  f_rhs = std::get<1>(rtpl);
  rtpl = maxCount(d2, f_rhs, "#_B");
  d2 = std::get<0>(rtpl);
  f_rhs = std::get<1>(rtpl);
  rtpl = maxCount(d2, f_rhs, "#");
  d2 = std::get<0>(rtpl);
  f_rhs = std::get<1>(rtpl);
  rtpl = maxCount(d2, f_rhs, "#");
  d2 = std::get<0>(rtpl);
  f_rhs = std::get<1>(rtpl);
  LOG_DEBUG("Removing state factors from f_rhs");
  rtpl = maxStateFactor(d2, f_rhs, "B");
  d2 = std::get<0>(rtpl);
  f_rhs = std::get<1>(rtpl);
  rtpl = maxStateFactor(d2, f_rhs, "A");
  d2 = std::get<0>(rtpl);
  f_rhs = std::get<1>(rtpl);

  LOG_INFO("Final result of max operation:");
  _StateIncrementIterator reslitr2(d1);
  _StateIncrementIterator resritr(d2);
  while(resritr.hasNext()) {
      const State& rs = resritr.next();
      const State& ls = reslitr2.next();
      EXPECT_TRUE(cpputil::approxEq(f_rhs->getValue(rs), f_lhs->getValue(ls)));
      LOG_INFO(ls << " " << f_lhs->getValue(ls));
  }
}

///
/// \brief Test variable elimination with a variable that is both proper and appears in counter
///
TEST(CounterTest, SharedProperCountTest) {
  srand(time(NULL));

  // f_LHS(A,#_A{A,B,C}) = f_RHS
  Domain d1, d2;
  d1 = boost::make_shared<_Domain>();
  d2 = boost::make_shared<_Domain>();
  d1->addStateFactor(0, 1, "A"); // 2 states
  d2->addStateFactor(0, 1, "A"); // 2 states
  d1->addStateFactor(0, 3, "#_A"); // 4 states
  d2->addStateFactor(0, 3, "#_A"); // 4 states

  FStateTable<Reward> f_lhs = boost::make_shared<_FStateTable<Reward>>(d1);
  FStateTable<Reward> f_rhs = boost::make_shared<_FStateTable<Reward>>(d2);
  _StateIncrementIterator litr(d1);
  while(litr.hasNext()) {
    const State& s = litr.next();
    Reward r = cpputil::randDouble();
    f_lhs->setValue(s, r);
    f_rhs->setValue(s, r);
  }

#if !NDEBUG
  LOG_DEBUG("f_lhs:");
  litr.reset();
  while(litr.hasNext()) {
    const State& s = litr.next();
    LOG_DEBUG(s << " " << f_lhs->getValue(s));
  }
#endif

  // manually eliminate A from counter scope (both non-shared counter and proper variable)
  Domain dn = removeCount(d1, "#_A");

  FStateTable<Reward> fn = boost::make_shared<_FStateTable<Reward>>(dn);
  _StateIncrementIterator sitr(dn);
  while(sitr.hasNext()) { // over #f(A,_A{B,C})
      const State& s = sitr.next();
      State p1(d1);
      for(Size i = 0; i < dn->getNumStateFactors(); i++) {
        p1.setFactor(i, s.getFactor(i));
      }
      // update based on referenced proper variable `A'
      Factor A_enabled = static_cast<Factor>(s.getFactor(0) != 0);
      p1.setFactor(1, p1.getFactor(1)+A_enabled); // min value in interval
      fn->setValue(s, f_lhs->getValue(p1));
  }
  // update domain, function
  d1 = dn;
  f_lhs = fn;

  // eliminate B from f_lhs #_A (non-shared counter)
  auto ltpl = maxCount(d1, f_lhs, "#_A");
  d1 = std::get<0>(ltpl);
  f_lhs = std::get<1>(ltpl);
  // eliminate C from f_lhs #_A (non-shared counter)
  ltpl = maxCount(d1, f_lhs, "#_A");
  d1 = std::get<0>(ltpl);
  f_lhs = std::get<1>(ltpl);
  LOG_DEBUG("Removing state factors from f_lhs");
  ltpl = maxStateFactor(d1, f_lhs, "A");
  d1 = std::get<0>(ltpl);
  f_lhs = std::get<1>(ltpl);

  //
  // different elimination order for f_RHS
  //

  // eliminate B from f_rhs #_A (non-shared counter)
  ltpl = maxCount(d2, f_rhs, "#_A");
  d2 = std::get<0>(ltpl);
  f_rhs = std::get<1>(ltpl);

  // manually eliminate A from counter scope (both non-shared counter and proper variable)
  dn = removeCount(d2, "#_A");

  fn = boost::make_shared<_FStateTable<Reward>>(dn);
  _StateIncrementIterator sitr2(dn);
  while(sitr2.hasNext()) { // over #f(A,_A{C})
      const State& s = sitr2.next();
      State p1(d2);
      for(Size i = 0; i < dn->getNumStateFactors(); i++) {
        p1.setFactor(i, s.getFactor(i));
      }
      // update based on referenced proper variable `A'
      Factor A_enabled = static_cast<Factor>(s.getFactor(0) != 0);
      p1.setFactor(1, p1.getFactor(1)+A_enabled); // min value in interval
      fn->setValue(s, f_rhs->getValue(p1));
  }
  // update domain, function
  d2 = dn;
  f_rhs = fn;

  LOG_DEBUG("Removing state factors from f_rhs");
  ltpl = maxStateFactor(d2, f_rhs, "A");
  d2 = std::get<0>(ltpl);
  f_rhs = std::get<1>(ltpl);
  // eliminate C from f_rhs #_A (non-shared counter)
  ltpl = maxCount(d2, f_rhs, "#_A");
  d2 = std::get<0>(ltpl);
  f_rhs = std::get<1>(ltpl);

  LOG_INFO("Final result of max operation:");
  _StateIncrementIterator reslitr(d1);
  _StateIncrementIterator resritr(d2);
  while(resritr.hasNext()) {
      const State& rs = resritr.next();
      const State& ls = reslitr.next();
      EXPECT_TRUE(cpputil::approxEq(f_rhs->getValue(rs), f_lhs->getValue(ls)));
      LOG_INFO(ls << " " << f_lhs->getValue(ls));
  }
}

///
/// \brief Basis tests of \a LiftedCounter from libcrl
///
TEST(CounterTest, LiftedCounterBasicTest) {
  _LiftedFactor a = {1,2,5,9,10};
  _LiftedFactor b = {1,5,2,10,9};
  EXPECT_TRUE(a == b);

  a.eraseStateFactor(100);
  EXPECT_TRUE(a == b);
  a.eraseStateFactor(9);
  EXPECT_FALSE(a == b);

  EXPECT_EQ(b.getRange().getMax(), 5);
}
