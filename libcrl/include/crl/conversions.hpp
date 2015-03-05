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
/// \brief write the \a MDP out to filename in SPUDD format.
///
void exportToSpudd(FactoredMDP mdp,
                   float gamma,
                   const std::string& problemName,
                   const std::string& filename
                   );

}

#endif /*CONVERSIONS_HPP_*/
