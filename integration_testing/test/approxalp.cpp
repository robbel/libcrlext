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

#include "crl/approx_alp.hpp"
#include "crl/env_sysadmin.hpp"
#include "crl/env_graphprop.hpp"
#include "crl/alp_gurobi.hpp"
#include "logger.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

//
// More complex integration tests that run iterative ALP constraint generation in big graphs
//

namespace {

/// \brief Test fixture
class ApproxALPTest : public ::testing::Test {
protected:
  ApproxALPTest() { }
//virtual ~ApproxALPTest() { }
  /// \brief Set up an example SysAdmin problem
  virtual void SetUp() override;
//virtual void TearDown() override;

  Domain _domain;
  FactoredMDP _fmdp;
  FactoredValueFunction _fval;
  ALPPlanner _planner;
};

void ApproxALPTest::SetUp() {
  sysadmin::Sysadmin thesys = buildSysadmin("ring", 4);
  _domain = thesys->getDomain();

  _fmdp = thesys->getFactoredMDP();
  LOG_INFO(_fmdp->T());

  _fval = boost::make_shared<_FactoredValueFunction>(_domain);
  const RangeVec& ranges = _domain->getStateRanges();

  // Basis function coverage
  for(Size fa = 0; fa < ranges.size(); fa++) { // assumption: DBN covers all domain variables
      auto I_o = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa}), State(_domain,0));
      _StateIncrementIterator sitr(I_o->getSubdomain());
      while(sitr.hasNext()) {
          auto I = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa}), sitr.next());
          _fval->addBasisFunction(std::move(I), 0.);
      }
  }
  // add the constant basis (to guarantee LP feasibility)
  //auto cfn = boost::make_shared<_ConstantFn<Reward>>(_domain);
  //_fval->addBasisFunction(std::move(cfn), 0.);
  // initialize ALP planner
  _planner = boost::make_shared<_ALPPlanner>(_fmdp, 0.9);
  _planner->setFactoredValueFunction(_fval); // this will be computed
}

} // anonymous ns

///
/// \brief Max-Plus test in the SysAdmin problem
///
TEST_F(ApproxALPTest, TestSysadminApproxArgmax) {
  srand(time(NULL));

  //
  // Run the ALP planner (with exact representation of non-linear max constraint)
  //

  long start_time = time_in_milli();
  int res = _planner->plan();
  long end_time = time_in_milli();
  LOG_INFO("FALP planner returned after " << end_time - start_time << "ms");
  EXPECT_EQ(res, 0) << "ALP " << (res == 1 ? "generateLP()" : "solve()") << " failed"; // else: lp successfully generated

  if(!res) { // success
    LOG_INFO("Results:");
    for(auto w : _fval->getWeight()) {
        LOG_INFO(" W: " << w);
    }

    //
    // Obtain approximate argMax via Max-Plus (using ALP solution weight vector)
    //

    _ApproxALP approxalp(_domain, _planner);
    // just take full ALP solution in this test
    approxalp.setWeights(_fval->getWeight());
    State maxjs(_domain);
    Action maxja(_domain);
    EXPECT_NO_THROW(approxalp.approxArgmax(maxjs, maxja));
    LOG_INFO("Max-Plus result (" << maxjs << ", " << maxja << ")\tRETURN: " << _fval->getQ(maxjs,maxja));

    //
    // Compare result with optimal argMax
    //

    FunctionSet<Reward> f_set = _fval->getMaxQ();
    auto qfns = f_set.getFunctions();
    _StateIncrementIterator sitr(_domain);
    double maxret = -numeric_limits<double>::infinity();
    while(sitr.hasNext()) {
      const State& s = sitr.next();
      double ret = 0.;
      for(auto& qf : qfns) {
          State ms = qf->mapState(s);
          ret += qf->eval(ms,Action());
      }
      if(ret > maxret) {
        maxret = ret;
      }
    }
    LOG_INFO("Versus optimal Q-function (" << qfns.size() << " local terms)\t\tRETURN: " << maxret);

    // write factor graph to .fg and .dot files
    boost::shared_ptr<dai::FactorGraph> fg = approxalp.getFactorGraph();
    fg->WriteToFile("test.fg");
    ofstream dotFile("test.dot");
    if(dotFile) {
      fg->printDot(dotFile);
    }
  }
  else {
    FAIL();
  }
  SUCCEED();
}

///
/// \brief Run iterative constraint generation in the SysAdmin ALP
///
TEST_F(ApproxALPTest, TestSysadminConstraintGeneration) {
  _planner->precompute(); // instead of running plan()

  // set up LP with objective
  gurobi::_LP lp(_domain);
  lp.generateObjective("IterCstrtGen", _planner->getAlpha());

  // add regularization/bound values
  double lambda = 10.; // maximum variable value
  lp.addAbsVariableBound(lambda, _planner->getAlpha());

  // initialize the Max-Plus constraint generator
  _ApproxALP approxalp(_domain, _planner);
  State maxjs(_domain);
  Action maxja(_domain);

  //
  // insert a couple of constraints
  //

  for(int i = 0; i < 10; i++) {
      approxalp.setWeights(_fval->getWeight());
      EXPECT_NO_THROW(approxalp.approxArgmax(maxjs, maxja));
      LOG_INFO("Max-Plus result (" << maxjs << ", " << maxja << ")\tRETURN: " << _fval->getQ(maxjs,maxja));
      // add new constraint to ALP
      lp.addStateActionConstraint(maxjs, maxja, _planner->getC(), _planner->getLRFs());
      int res = lp.solve(_fval.get());
      EXPECT_EQ(res, 0) << "ALP solve() failed";
  }

  // TODO: check whether js,ja pair has already been inserted
}


///
/// \todo
///
TEST(ApproxALPIntegrationTest, TestGraphpropApproxArgmax) {

  // TODO (BigState!)

  SUCCEED();
}
