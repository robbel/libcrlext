/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include "crl/bigindex.hpp"

namespace crl {

// Note: can be removed
void BigState::print(std::ostream& os) const {
	for (Size i=0; i<size(); i++) {
		if (i != 0)
			os << " ";
		os << getFactor(i);
	}
}

State& BigState::operator=(const State& rhs) noexcept {
  const BigState* p = dynamic_cast<const BigState*>(&rhs);
  if(p) {
      return operator=(*p);
  }
//  else {
//  }

  return *this;
}

State& BigState::operator=(State&& rhs) noexcept {
  BigState* p = dynamic_cast<BigState*>(&rhs);
  if(p) {
      return operator=(std::move(*p));
  }
//  else {
//  }

  return *this;
}


} // namespace crl
