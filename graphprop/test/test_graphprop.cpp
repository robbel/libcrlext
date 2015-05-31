/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <gtest/gtest.h>

#include "crl/env_graphprop.hpp"
#include "logger.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;
using namespace graphprop;

///
/// \brief Placeholder for some GraphProp experiments
///
TEST(GraphPropTest, AdjacencyTransposeTest) {
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

TEST(GraphPropTest, BasicGraphPropTest) {
  SUCCEED();
}
