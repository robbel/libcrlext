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
#include "logger.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

//
// More complex integration tests that run iterative ALP constraint generation in big graphs
//

TEST(ApproxALPIntegrationTest, TestSysadminApproxArgmax) {
  srand(time(NULL));

  //
  // Set up a SysAdmin problem
  //

  sysadmin::Sysadmin thesys = buildSysadmin("ring", 4);
  Domain domain = thesys->getDomain();

  FactoredMDP fmdp = thesys->getFactoredMDP();
  LOG_INFO(fmdp->T());

  FactoredValueFunction fval = boost::make_shared<_FactoredValueFunction>(domain);
  const RangeVec& ranges = domain->getStateRanges();

  for(Size fa = 0; fa < ranges.size(); fa+=2) { // assumption: DBN covers all domain variables
      auto I_o = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa,fa+1}), State(domain,0));
      _StateIncrementIterator sitr(I_o->getSubdomain());
      while(sitr.hasNext()) {
          auto I = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa,fa+1}), sitr.next());
          fval->addBasisFunction(std::move(I), 0.);
      }
  }

  //
  // Run the ALP planner (with exact representation of non-linear max constraint)
  //

  ALPPlanner planner = boost::make_shared<_ALPPlanner>(fmdp, 0.9);

  long start_time = time_in_milli();
  planner->setFactoredValueFunction(fval); // this will be computed
  int res = planner->plan();
  long end_time = time_in_milli();
  LOG_INFO("FALP planner returned after " << end_time - start_time << "ms");
  EXPECT_EQ(res, 0) << "ALP " << (res == 1 ? "generateLP()" : "solve()") << " failed"; // else: lp successfully generated

  if(!res) { // success
    LOG_INFO("Results:");
    for(auto w : fval->getWeight()) {
        LOG_INFO(" W: " << w);
    }

    //
    // Obtain approximate argMax via Max-Plus (using ALP solution weight vector)
    //

    _ApproxALP approxalp(domain, planner);
    // just take full ALP solution in this test
    approxalp.setWeights(fval->getWeight());
    State maxjs(domain);
    Action maxja(domain);
    EXPECT_NO_THROW(approxalp.approxArgmax(maxjs, maxja));
    LOG_INFO("Max-Plus result (" << maxjs << ", " << maxja << ") RETURN: " << fval->getQ(maxjs,maxja));

    //
    // Compare result with optimal argMax
    //
    FunctionSet<Reward> f_set = fval->getMaxQ();
    auto qfns = f_set.getFunctions();
    _StateIncrementIterator sitr(domain);
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
    LOG_INFO("Versus optimal Q-function (" << qfns.size() << " local terms)\tRETURN: " << maxret);

    // write factor graph to .fg and .dot files
    boost::shared_ptr<dai::FactorGraph> fg = approxalp.getFactorGraph();
    fg->WriteToFile("test.fg");
    ofstream dotFile("test.dot");
    if(dotFile) {
      fg->printDot(dotFile);
    }
  }

  SUCCEED();
}

TEST(ApproxALPIntegrationTest, TestGraphpropApproxArgmax) {

  // TODO (BigState!)

  SUCCEED();
}
