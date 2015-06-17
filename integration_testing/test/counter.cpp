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
#include "logger.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

namespace crl {

template<class T>
using FStateTable = boost::shared_ptr<_FStateTable<T>>;

}

namespace {

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

  return std::make_tuple(dn,fn);
}

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

  return std::make_tuple(dn,fn);
}

} // anonymous ns


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
