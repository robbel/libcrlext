/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth
    Copyright 2015 Philipp Robbel

    This file is part of CRL:RL-Glue.

    CRL:RL-Glue is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CRL:RL-Glue is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CRL:RL-Glue.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef GLUE_ENV_HPP_
#define GLUE_ENV_HPP_

#include <crl/crl.hpp>

namespace crl {

///
/// \brief Construct and obtain domain corresponding to desired crl::Environment
/// \see env_init()
///
Domain getCRLEnvironmentDomain();

///
/// \brief Construct and obtain desired \a crl::Environment from a \a crl::Domain
/// \see env_init()
///
Environment getCRLEnvironment(Domain domain);

}

#endif /*GLUE_HPP_*/
