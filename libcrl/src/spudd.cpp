/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <iostream>
#include "crl/spudd.hpp"
#include "cpputil.hpp"
#include "logger.hpp"

#include "MDP.h" // main SPUDD header

using namespace std;
using namespace crl;

namespace crl {

Action _SpuddPolicy::getAction(const State& s) {
    return Action();
};

} // namespace crl
