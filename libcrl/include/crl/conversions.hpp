/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef CONVERSIONS_HPP_
#define CONVERSIONS_HPP_

#include <string>
#include "crl/crl.hpp"

namespace crl {

// forward declarations
class _FactoredMDP;
typedef boost::shared_ptr<_FactoredMDP> FactoredMDP;

///
/// \brief write the \a FactoredMDP representing a specific problem to a file in SPUDD format.
/// \return 0 iff successful
/// \note This function is ported over from a personal contribution to the Multiagent decision process (MADP) Toolbox and re-released under the LGPL
///
int exportToSpudd(FactoredMDP mdp,
                   Domain domain,
                   float gamma,
                   const std::string& problemName,
                   const std::string& filename
                   );

/// \brief Convert the supplied (joint) \a RLType into a string, given variable names from a \a Domain.
std::string concat(const crl::RLType& jt, const StrVec& names);
/// \brief Return the array representation of a \a State
FactorVec resolve(const Domain& dom, const State& s);
/// \brief Return the array representation of an \a Action
FactorVec resolve(const Domain& dom, const Action& a);

} // namespace crl

#endif /*CONVERSIONS_HPP_*/
