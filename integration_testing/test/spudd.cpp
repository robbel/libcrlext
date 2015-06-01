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

#include "crl/env_sysadmin.hpp"
#include "crl/env_graphprop.hpp"
#include "crl/conversions.hpp"
#include "logger.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

//
// More complex integration test that runs the SPUDD exporter on the SysAdmin and GraphProp problems
//

TEST(SPUDDIntegrationTest, TestSysadmin) {
  srand(time(NULL));

  try {
    sysadmin::Sysadmin thesys = buildSysadmin("ring", 4);
    Domain domain = thesys->getDomain();

    FactoredMDP fmdp = thesys->getFactoredMDP();
    LOG_INFO(fmdp->T());

    EXPECT_EQ(exportToSpudd(fmdp, domain, 0.99, "sysadmin_ring_4", "sysadmin_ring_4.spudd"), 0);

  }
  catch(const cpputil::Exception& e) {
    cerr << e << endl;
    FAIL();
  }

  SUCCEED();
}

TEST(SPUDDIntegrationTest, TestGraphProp) {
  srand(time(NULL));

  try {
    string cfg = "../data/default.xml";
    string data = "../data/Gnp100.txt";
    ifstream iscfg(cfg);
    ifstream isdat(data);
    graphprop::GraphProp thegrp;

    if(!(thegrp = readGraphProp(iscfg, isdat))) {
      LOG_ERROR("Error while reading from " << cfg << " or " << data);
      FAIL();
    }

    Domain domain = thegrp->getDomain();
    FactoredMDP fmdp = thegrp->getFactoredMDP();
    LOG_INFO(fmdp->T());

    EXPECT_EQ(exportToSpudd(fmdp, domain, 0.99, "graphprop_gnp_100", "graphprop_gnp_100.spudd"), 0);

  }
  catch(const cpputil::Exception& e) {
    cerr << e << endl;
    FAIL();
  }

  SUCCEED();
}
