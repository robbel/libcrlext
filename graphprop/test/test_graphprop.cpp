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

#include "crl/env_graphprop.hpp"
#include "logger.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;
using namespace graphprop;

namespace {

//
// Test fixtures
//

class GraphPropTest : public ::testing::Test {
protected:
  GraphPropTest() { }
//virtual ~GraphPropTest() { }
  /// \brief Load an example GraphProp instance from filesystem
  virtual void SetUp() override;
//virtual void TearDown() override;

  Domain _domain;
  FactoredMDP _fmdp;
};

void GraphPropTest::SetUp() {
  try {
    string cfg = "../cfgs/default.xml";
    string data = "../data/Gnp100.txt";
    ifstream iscfg(cfg);
    ifstream isdat(data);
    graphprop::GraphProp thegrp;

    if(!(thegrp = readGraphProp(iscfg, isdat))) {
      LOG_ERROR("Error while reading from " << cfg << " or " << data);
      FAIL();
    }

    _domain = thegrp->getDomain();
    _fmdp = thegrp->getFactoredMDP();
  }
  catch(const cpputil::Exception& e) {
    cerr << e << endl;
    FAIL();
  }
}

} // anonymous ns


///
/// \brief Placeholder for some GraphProp experiments
///
TEST(GraphPropBasicTest, AdjacencyTransposeTest) {
  Domain domain = boost::make_shared<_Domain>();
  for(int i = 0; i < 100; i++) {
    domain->addStateFactor(0,1);
  }

  _AdjacencyMap tbl(domain);
  // define some rows
  for(Size j = 0; j < 100; j++) {
      tbl.setValue(0,j,2);
      tbl.setValue(10,j,3);
      tbl.setValue(99,j,4);
  }

  auto tpose = tbl.transpose();
  tbl.values() = tpose;
  for(Size j = 0; j < 100; j++) {
      EXPECT_EQ(tbl.getValue(j,0), 2);
      EXPECT_EQ(tbl.getValue(j,10), 3);
      EXPECT_EQ(tbl.getValue(j,99), 4);
  }
}

TEST_F(GraphPropTest, DBNTest) {
  // print DBN to stdout
  const _DBN& dbn = _fmdp->T();
  LOG_DEBUG(dbn);

  // verify that all probability distributions are normalized
  FactorIterator fitr = dbn.factors();
  while(fitr->hasNext()) {
      // determine relevant action subset for current state factor
      const DBNFactor& sf = fitr->next();
      _StateActionIncrementIterator saitr(sf->getSubdomain());
      while(saitr.hasNext()) {
          const std::tuple<State,Action>& sa = saitr.next();
          const State s = std::get<0>(sa);
          const Action a = std::get<1>(sa);
          const ProbabilityVec& vec = sf->T(s,a);
          double d = std::accumulate(vec.begin(),vec.end(),0.);
          EXPECT_EQ(d,1.);
      }
  }
}

TEST_F(GraphPropTest, LRFTest) {
  const RewardRange& range = _domain->getRewardRange();
  const Reward rmin = range.getMin();
  const Reward rmax = range.getMax()+std::numeric_limits<double>::epsilon();
  LOG_DEBUG("rmin/rmax: " << rmin << " " << rmax);

  const std::vector<DiscreteFunction<Reward>>& lrfs = _fmdp->getLRFs();
  for(auto& f : lrfs) {
      _StateActionIncrementIterator saitr(f->getSubdomain());
      while(saitr.hasNext()) {
          const std::tuple<State,Action>& sa = saitr.next();
          const State s = std::get<0>(sa);
          const Action a = std::get<1>(sa);
          const Reward r = f->eval(s,a);
          EXPECT_TRUE(cpputil::in_interval(r,rmin,rmax));
      }
  }
}
