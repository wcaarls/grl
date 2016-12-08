/** \file q_policy.cpp
 * \brief Q policy mapping source file. 
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

#include <grl/mappings/q_policy.h>

using namespace grl;

REGISTER_CONFIGURABLE(QPolicyMapping)

void QPolicyMapping::request(ConfigurationRequest *config)
{
  config->push_back(CRP("policy", "policy/discrete/q", "Q-value based policy", policy_));
}

void QPolicyMapping::configure(Configuration &config)
{
  policy_ = (QPolicy*)config["policy"].ptr();
}

void QPolicyMapping::reconfigure(const Configuration &config)
{
}

double QPolicyMapping::read(const Vector &in, Vector *result) const
{
  *result = VectorConstructor(policy_->value(in));
  return (*result)[0];
}
