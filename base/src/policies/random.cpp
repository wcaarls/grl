/** \file random.cpp
 * \brief Discrete and continuous random policies source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
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

#include <grl/policies/random.h>

using namespace grl;

REGISTER_CONFIGURABLE(RandomPolicy)
REGISTER_CONFIGURABLE(RandomDiscretePolicy)

void RandomPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("output_min", "vector.action_min", "Lower action limit", min_, CRP::System));
  config->push_back(CRP("output_max", "vector.action_max", "Upper action limit", max_, CRP::System));
}

void RandomPolicy::configure(Configuration &config)
{
  min_ = config["output_min"].v();
  max_ = config["output_max"].v();
  
  if (min_.size() != max_.size())
    throw bad_param("policy/random:{min,max}");
}

void RandomPolicy::reconfigure(const Configuration &config)
{
}

RandomPolicy *RandomPolicy::clone() const
{
  return new RandomPolicy(*this);
}

void RandomPolicy::act(const Vector &in, Vector *out) const
{
  out->resize(min_.size());
  Rand *rand = RandGen::instance();
  
  for (size_t ii=0; ii < min_.size(); ++ii)
    (*out)[ii] = rand->getUniform(min_[ii], max_[ii]);
}

void RandomDiscretePolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
}

void RandomDiscretePolicy::configure(Configuration &config)
{
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
}

void RandomDiscretePolicy::reconfigure(const Configuration &config)
{
}

RandomDiscretePolicy *RandomDiscretePolicy::clone() const
{
  return new RandomDiscretePolicy(*this);
}

void RandomDiscretePolicy::act(const Vector &in, Vector *out) const
{
  *out = discretizer_->at(in, RandGen::getInteger(discretizer_->size(in)));
}
