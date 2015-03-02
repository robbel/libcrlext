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

using namespace std;
using namespace crl;

namespace crl {

///
/// \brief write the \a MDP out to filename in SPUDD format.
///
void exportToSpudd(MDP mdp, Domain domain, const string& filename);

}

#endif /*CONVERSIONS_HPP_*/
