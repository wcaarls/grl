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
  config->push_back(CRP("sigma", "Standard deviation of exploration distribution", sigma_, CRP::Configuration));
  
  config->push_back(CRP("output_min", "vector.action_min", "Lower limit on outputs", min_, CRP::System));
  config->push_back(CRP("output_max", "vector.action_max", "Upper limit on outputs", max_, CRP::System));

  config->push_back(CRP("projector", "projector.observation", "Projects observations onto representation space", projector_));
  config->push_back(CRP("representation", "representation.action", "Action representation", representation_));
}

void ActionPolicy::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  
  sigma_ = config["sigma"];
  min_ = config["output_min"];
  max_ = config["output_max"];
  
  if (min_.size() != max_.size() || !min_.size())
    throw bad_param("policy/action:{output_min,output_max}");
  
  if (!sigma_.size())
    sigma_ = ConstantVector(min_.size(), 0.);
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
  if (!out->size())
    *out = (min_+max_)/2;
  
  for (size_t ii=0; ii < out->size(); ++ii)
  {
    if (sigma_[ii])
      (*out)[ii] += RandGen::getNormal(0., sigma_[ii]);
      
    (*out)[ii] = fmin(fmax((*out)[ii], min_[ii]), max_[ii]);
  }    
}

void ActionProbabilityPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("discretizer", "discretizer", "Action discretizer", discretizer_));
  config->push_back(CRP("projector", "projector", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation", "Action-probability representation", representation_));
}

void ActionProbabilityPolicy::configure(Configuration &config)
{
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  discretizer_->options(&variants_);
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
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
