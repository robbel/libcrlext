/*
    Copyright 2015 Philipp Robbel

    TODO: ADD LICENSE
 */

#ifndef CONVERSIONS_HPP_
#define CONVERSIONS_HPP_

#include <string>
#include "crl/crl.hpp"
#include "crl/factor_learner.hpp"


namespace crl {

///
/// \brief write the \a FactoredMDP representing a specific problem to a file in SPUDD format.
///
void exportToSpudd(FactoredMDP mdp,
                   Domain domain,
                   float gamma,
                   const std::string& problemName,
                   const std::string& filename
                   );

}

#endif /*CONVERSIONS_HPP_*/
