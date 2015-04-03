/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include "crl/alp.hpp"
#include <lpsolve/lp_lib.h>

using namespace std;

namespace crl {

void lp_demo() {
    lprec *lp;

    lp = make_lp(0,4);

        /* ... */

    delete_lp(lp);
}

void _ALPPlanner::plan() {

}

} // namespace crl
