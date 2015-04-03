/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include "crl/alp.hpp"
#include "crl/alp_lpsolve.hpp"

using namespace std;

namespace crl {

int _ALPPlanner::plan() {
  lpsolve::lp_demo_2();
  int res = lpsolve::lp_demo();
  return res;
}

} // namespace crl
