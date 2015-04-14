/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <gtest/gtest.h>

#include "crl/alp.hpp"
#include "crl/alp_lpsolve.hpp"
#include "crl/env_sysadmin.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

//
// More complex integration test that runs the ALP solver on the SysAdmin environment
//

namespace {

} // anonymous ns

TEST(ALPIntegrationTest, TestSysadmin) {
  srand(time(NULL));

  sysadmin::Sysadmin thesys = buildSysadmin("ring", 1);
  Domain domain = thesys->getDomain();

  FactoredMDP fmdp = thesys->getFactoredMDP();
  std::cout << fmdp->T() << std::endl;

  // create a basis
  FactoredValueFunction fval = boost::make_shared<_FactoredValueFunction>(domain);
  const RangeVec& ranges = domain->getStateRanges();
  for(Size fa = 0; fa < ranges.size(); fa++) { // assumption: DBN covers all domain variables
      // place one indicator basis on each possible factor value
      for(Factor fv=0; fv<=ranges[fa].getSpan(); fv++) {
          State dummy_s; // we don't care about actual domain here
          dummy_s.setIndex(ranges[fa].getMin()+fv);
          auto I = boost::make_shared<_Indicator<Reward>>(domain);
          I->addStateFactor(fa);
          I->setState(dummy_s);
          fval->addBasisFunction(I, 0.);
      }
  }

  // add the constant function
//  auto C  = boost::make_shared<_ConstantFn<Reward>>(domain); // TODO currently not supported, needs Backproj template specialization
  // add some indicators
//  auto I1 = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({0}), State(domain,0));
//  auto I2 = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({2}), State(domain,0));
//  auto I3 = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({4}), State(domain,0));
//  auto I4 = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({6}), State(domain,0));

  // run the ALP planner
  _ALPPlanner planner(fmdp, 0.99);
//  fval->addBasisFunction(C,  0.);
//  fval->addBasisFunction(I1, 0.);
//  fval->addBasisFunction(I2, 0.);
//  fval->addBasisFunction(I3, 0.);
//  fval->addBasisFunction(I4, 0.);

  long start_time = time_in_milli();
  planner.setFactoredValueFunction(fval); // this will be computed
  int res = planner.plan();
  long end_time = time_in_milli();
  std::cout << "[DEBUG]: FALP planner returned after " << end_time - start_time << "ms" << std::endl;

  EXPECT_EQ(res, 0) << "ALP planner failed with error code " << res; // else: lp successfully generated

  if(!res) { // success
    for(auto w : fval->getWeight()) {
        std::cout << "w: " << w << std::endl;
    }
  }
}
