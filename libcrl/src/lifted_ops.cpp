/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <iostream>
#include "crl/lifted_ops.hpp"

using namespace std;

namespace crl {

const std::size_t _LiftedFactor::EMPTY_HASH = 0;

bool _LiftedFactor::eraseStateFactor(Size i) {
  bool ret = false;
  SizeVec::iterator it = std::lower_bound(_state_dom.begin(), _state_dom.end(), i);
  if(it != _state_dom.end() && (*it) == i) {
    _state_dom.erase(it);
    _dom_hash = boost::hash_range(_state_dom.begin(), _state_dom.end());
    ret = true;
  }
  return ret;
}

std::ostream& operator<<(std::ostream &os, const _LiftedFactor& factor) {
  const SizeVec& fs = factor.getStateFactors();
  os << "#{";
  for(auto f : fs) {
    os << f << ",";
  }
  os << "}";
  return os;
}

} // namespace crl
