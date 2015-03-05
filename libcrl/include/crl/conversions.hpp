/*
    Copyright 2015 Philipp Robbel

    TODO: ADD LICENSE
 */

#ifndef CONVERSIONS_HPP_
#define CONVERSIONS_HPP_

#include <iostream>
#include <string>
#include <cassert>
#include "crl/crl.hpp"
#include "crl/flat_tables.hpp"

using namespace crl;

namespace crl {

///
/// \brief write the \a MDP out to filename in SPUDD format.
///
void exportToSpudd(MDP mdp, Domain domain, const std::string& filename, const std::string& problemName);

}

#endif /*CONVERSIONS_HPP_*/
