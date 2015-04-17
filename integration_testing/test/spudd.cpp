/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <gtest/gtest.h>

#include "crl/env_sysadmin.hpp"
#include "crl/conversions.hpp"
#include "logger.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

//
// More complex integration test that runs the SPUDD exporter on the SysAdmin problem
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
