/** \file action.cpp
 * \brief Action and action-probability policies policies source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-16
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

#include <grl/policies/action.h>

using namespace grl;

REGISTER_CONFIGURABLE(ActionPolicy)
REGISTER_CONFIGURABLE(ActionProbabilityPolicy)

void ActionPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("min", "Lower action limit", min_));
  config->push_back(CRP("max", "Upper action limit", max_));
  config->push_back(CRP("sigma", "Standard deviation of exploration distribution", sigma_, CRP::Configuration));

  config->push_back(CRP("projector", "projector", "Projects observations onto representation space", projector_));
  config->push_back(CRP("representation", "representation", "Action representation", representation_));
}

void ActionPolicy::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (ParameterizedRepresentation*)config["representation"].ptr();
  
  min_ = config["min"];
  max_ = config["max"];
  sigma_ = config["sigma"];
  
  if (min_.size() != max_.size())
    throw bad_param("policy/action:{min,max}");

  if (sigma_.empty())
    sigma_.resize(min_.size(), 0.);
    
  if (sigma_.size() != min_.size())
    throw bad_param("policy/action:sigma");
}

void ActionPolicy::reconfigure(const Configuration &config)
{
}

ActionPolicy *ActionPolicy::clone() const
{
  ActionPolicy *cpp = new ActionPolicy(*this);
  cpp->projector_ = projector_->clone();
  cpp->representation_ = representation_->clone();
  return cpp;
}

void ActionPolicy::act(const Vector &in, Vector *out) const
{
  ProjectionPtr p = projector_->project(in);
  representation_->read(p, out);
  
  // Some representations may not always return a value.
  if (out->empty())
    *out = (max_-min_)/2;
  
  if (!min_.empty())
  {
    if (out->size() != min_.size())
      throw bad_param("policy/action:{min,max}");
    
    for (size_t ii=0; ii < out->size(); ++ii)
    {
      if (sigma_[ii])
        (*out)[ii] += RandGen::getNormal(0., sigma_[ii]);
      (*out)[ii] = fmin(fmax((*out)[ii], min_[ii]), max_[ii]);
    }
  }
}

void ActionProbabilityPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("discretizer", "discretizer", "Action discretizer", discretizer_));
  config->push_back(CRP("projector", "projector", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation/parameterized", "Action-probability representation", representation_));
}

void ActionProbabilityPolicy::configure(Configuration &config)
{
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  discretizer_->options(&variants_);
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (ParameterizedRepresentation*)config["representation"].ptr();
}

void ActionProbabilityPolicy::reconfigure(const Configuration &config)
{
}

ActionProbabilityPolicy *ActionProbabilityPolicy::clone() const
{
  ActionProbabilityPolicy *dpp = new ActionProbabilityPolicy(*this);
  dpp->discretizer_ = discretizer_->clone();
  dpp->projector_ = projector_->clone();
  dpp->representation_ = representation_->clone();
  return dpp;
}

void ActionProbabilityPolicy::act(const Vector &in, Vector *out) const
{
  std::vector<ProjectionPtr> projections;
  projector_->project(in, variants_, &projections);

  Vector dist(variants_.size()), v;
  
  for (size_t ii=0; ii < variants_.size(); ++ii)
    dist[ii] = representation_->read(projections[ii], &v);
    
  *out = variants_[sample(dist)];
}
