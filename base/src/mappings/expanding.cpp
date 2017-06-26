/** \file expanding.cpp
 * \brief Expanding mapping source file. 
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-06-26
 *
 * \copyright \verbatim
 * Copyright (c) 2017, Wouter Caarls
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

#include <grl/mappings/expanding.h>

using namespace grl;

REGISTER_CONFIGURABLE(ExpandingMapping)

void ExpandingMapping::request(ConfigurationRequest *config)
{
  config->push_back(CRP("policy", "mapping/policy", "Optional policy used to calculate action", policy_, true));
  config->push_back(CRP("discretizer", "discretizer.action", "Discretizer to convert discrete into continuous action", discretizer_));
}

void ExpandingMapping::configure(Configuration &config)
{
  policy_ = (Policy*)config["policy"].ptr();
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
}

void ExpandingMapping::reconfigure(const Configuration &config)
{
}

double ExpandingMapping::read(const Vector &in, Vector *result) const
{
  Vector observation;
  size_t action;

  if (policy_)
  {
    Action a;
  
    observation = in;
    policy_->act(in, &a);
    action = (size_t) a[0];
  }
  else
  {
    observation = in.head(in.size()-1);
    action = (size_t) in.tail(1)[0];
  }

  *result = discretizer_->at(observation, action);
  
  return (*result)[0];
}
