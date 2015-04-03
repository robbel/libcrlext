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
    if(lp == nullptr) {
        std::cerr << "Unable to create new LP model" << std::endl;
        return;
    }

    set_verbose(lp, NORMAL);
//  set_verbose(lp, IMPORTANT);

    /* Some notes on lpsolve:
       If the model is build column by column, then it is strongly suggested to use add_columnex instead of add_column because
            add_columnex gives the possibility to only supply the non-zero elements and that speeds up building the model considerably,
            especially if the matrix is sparse (a lot of zero elements).
       Version 5 has a new API call set_add_rowmode that makes add_constraint, str_add_constraint spectacular faster,
       If the model is build row by row, then it is strongly suggested to use add_constraintex instead of add_constraint because add_constraintex
            gives the possibility to only supply the non-zero elements and that speeds up building the model considerably, especially if the matrix
            is sparse (a lot of zero elements).
       Also see http://web.mit.edu/lpsolve/examples/Demo.java and http://cgi.csc.liv.ac.uk/~anshul/BIMatrix/lpsolve/leader.c
     */

    delete_lp(lp);
}

void _ALPPlanner::plan() {

}

} // namespace crl
