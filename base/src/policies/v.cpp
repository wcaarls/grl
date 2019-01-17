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
  
  model_ = (ObservationModel*)config["model"].ptr();
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  sampler_ = (Sampler*)config["sampler"].ptr();
}

void VPolicy::reconfigure(const Configuration &config)
{
}

double VPolicy::value(const Observation &in) const
{
  Vector v;
  representation_->read(projector_->project(in), &v);
  
  if (v.size())
    return v[0];
  else
    return 0;
}

void VPolicy::values(const Observation &in, LargeVector *out) const
{
  out->resize(discretizer_->size(in));
  
  size_t aa=0;
  for (Discretizer::iterator it = discretizer_->begin(in); it != discretizer_->end(); ++it, ++aa)
  {
    Observation next;
    double reward;
    int terminal;
    
    double tau = model_->step(in, *it, &next, &reward, &terminal);
    
    if (next.size())
    {
      if (terminal != 2)
      {
        Vector value;
        reward += reward + pow(gamma_, tau)*representation_->read(projector_->project(next), &value);
      }
      
      (*out)[aa] = reward;
    }
  }
}

void VPolicy::act(const Observation &in, Action *out) const
{
  LargeVector v;
  ActionType at;

  values(in, &v);
  size_t action = sampler_->sample(v, &at);
  
  *out = discretizer_->at(in, action);
  out->type = at;
}

void VPolicy::distribution(const Observation &in, const Action &prev, LargeVector *out) const
{
  LargeVector v;

  values(in, &v);
  sampler_->distribution(v, out);
}
