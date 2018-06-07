/** \file value.cpp
 * \brief Policy value mapping source file. 
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-06-01
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Wouter Caarls
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

#include <grl/mappings/value.h>

using namespace grl;

REGISTER_CONFIGURABLE(ValueMapping)

void ValueMapping::request(ConfigurationRequest *config)
{
  config->push_back(CRP("policy", "mapping/policy/discrete/value", "Value based policy", policy_));
}

void ValueMapping::configure(Configuration &config)
{
  policy_ = (ValuePolicy*)config["policy"].ptr();
}

void ValueMapping::reconfigure(const Configuration &config)
{
}

double ValueMapping::read(const Vector &in, Vector *result) const
{
  Observation obs = in;
  *result = VectorConstructor(policy_->value(obs));
  return (*result)[0];
}
