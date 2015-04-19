/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef SPUDD_HPP_
#define SPUDD_HPP_

#include "crl/crl.hpp"

namespace crl {

/**
 * A wrapper for a pre-computed SPUDD policy
 */
class _SpuddPolicy : public _Policy {
public:
    _SpuddPolicy();
    virtual ~_SpuddPolicy() { }

    /// \brief Return the action encoded in the pre-computed policy
    virtual Action getAction(const State& s) override;
};

} // namespace crl

#endif /*SPUDD_HPP_*/
