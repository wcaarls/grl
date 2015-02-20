/** \file parameterized.cpp
 * \brief Parameterized action policy source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-13
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

#include <grl/policies/parameterized.h>

using namespace grl;

REGISTER_CONFIGURABLE(ParameterizedActionPolicy)

void ParameterizedActionPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("min", "Lower action limit", min_, CRP::System));
  config->push_back(CRP("max", "Upper action limit", max_, CRP::System));
  config->push_back(CRP("sigma", "Standard deviation of exploration distribution", sigma_, CRP::Configuration));

  config->push_back(CRP("projector", "projector", "Projects observations onto representation space", projector_));
  config->push_back(CRP("representation", "representation/parameterized", "Action representation", representation_));
}

void ParameterizedActionPolicy::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (ParameterizedRepresentation*)config["representation"].ptr();
  
  min_ = config["min"];
  max_ = config["max"];
  sigma_ = config["sigma"];
  
  if (min_.size() != max_.size())
    throw bad_param("policy/parameterized/action:{min,max}");
    
  if (sigma_.empty())
    sigma_.resize(min_.size(), 0.);

  if (sigma_.size() != min_.size())
    throw bad_param("policy/parameterized/action:sigma");
}

void ParameterizedActionPolicy::reconfigure(const Configuration &config)
{
}

ParameterizedActionPolicy *ParameterizedActionPolicy::clone() const
{
  ParameterizedActionPolicy *cpp = new ParameterizedActionPolicy(*this);
  cpp->projector_ = projector_->clone();
  cpp->representation_ = representation_->clone();
  return cpp;
}

void ParameterizedActionPolicy::act(const Vector &in, Vector *out) const
{
  ProjectionPtr p = projector_->project(in);
  representation_->read(p, out);
  
  if (!min_.empty())
  {
    if (out->size() != min_.size())
      throw bad_param("policy/parameterized/action:{min,max}");
    
    for (size_t ii=0; ii < out->size(); ++ii)
    {
      if (sigma_[ii])
        (*out)[ii] += RandGen::getNormal(0., sigma_[ii]);
      (*out)[ii] = fmin(fmax((*out)[ii], min_[ii]), max_[ii]);
    }
  }
}
