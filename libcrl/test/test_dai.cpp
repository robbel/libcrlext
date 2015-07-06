/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <gtest/gtest.h>

#include "crl/approx_alp.hpp"
#include "cpputil.hpp"

///
/// \brief Placeholder for some libDAI experiments
///
TEST(LibDAITest, BasicLibDAITest) {
  int res = crl::testing::maxplus_demo();
  EXPECT_EQ(res, 0) << "max-plus failed";
}
