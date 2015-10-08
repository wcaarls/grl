/** \file v.cpp
 * \brief V policy source file. 
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

#include <grl/policies/v.h>

using namespace grl;

REGISTER_CONFIGURABLE(VPolicy)

void VPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  
  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("model", "observation_model", "Observation model", model_));
  config->push_back(CRP("projector", "projector.observation", "Projects observations onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/state", "State-value representation", representation_));
  config->push_back(CRP("sampler", "sampler", "Samples actions from state-values", sampler_));
}

void VPolicy::configure(Configuration &config)
{
  gamma_ = config["gamma"];

  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  discretizer_->options(&variants_);
  
  model_ = (ObservationModel*)config["model"].ptr();
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  sampler_ = (Sampler*)config["sampler"].ptr();
}

void VPolicy::reconfigure(const Configuration &config)
{
}

VPolicy *VPolicy::clone() const
{
  VPolicy *vp = new VPolicy(*this);
  vp->discretizer_ = discretizer_->clone();
  vp->model_ = model_->clone();
  vp->projector_ = projector_->clone();
  vp->representation_ = representation_->clone();
  vp->sampler_ = sampler_->clone();
  
  return vp;
}

void VPolicy::values(const Vector &in, Vector *out) const
{
  out->resize(variants_.size());

  for (size_t ii=0; ii < variants_.size(); ++ii)
  {
    Vector next;
    double reward;
    int terminal;
    
    model_->step(in, variants_[ii], &next, &reward, &terminal);
    
    if (next.size())
    {
      if (!terminal)
      {
        Vector value;
        reward += reward + gamma_*representation_->read(projector_->project(next), &value);
      }
      
      (*out)[ii] = reward;
    }
  }
}

void VPolicy::act(const Vector &in, Vector *out) const
{
  Vector v;
  
  values(in, &v);
  size_t action = sampler_->sample(v);
  
  *out = variants_[action];
}
            