/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <gtest/gtest.h>

#include "crl/alp_gurobi.hpp"

///
/// \brief Placeholder for some Gurobi experiments
///
TEST(GurobiTest, BasicGurobiTest) {
  int res = gurobi::testing::lp_demo();
  EXPECT_EQ(res, 0) << "no optimal solution found"; // else: optimal solution found, no error
}
