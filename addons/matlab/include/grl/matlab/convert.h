/** \file convert.h
 * \brief Conversions between Matlab and grl datatypes header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-17
 *
 * \copyright \verbatim
 * Copyright (c) 2015, Wouter Caarls
 * All rights reserved.
 *
 * This file is part of GRL, the Generic Reinforcement Learning library.
 *
 * GRL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * \endverbatim
 */
 
#ifndef GRL_MATLAB_CONVERT_H_
#define GRL_MATLAB_CONVERT_H_

#include <mex.h>
#include <grl/configuration.h>

mxArray *vectorToArray(const grl::Vector &v);
grl::Vector arrayToVector(const mxArray *pm);

void structToConfig(const mxArray *pm, grl::Configuration &config);
mxArray *taskSpecToStruct(const grl::Configuration &config);

#endif /* GRL_MATLAB_CONVERT_H_ */
