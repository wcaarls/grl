/** \file ann.cpp
 * \brief Artificial neural network representation source file.
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

#include <string.h>
#include <grl/representations/ann.h>

#define MAX_NODES 64

using namespace grl;

REGISTER_CONFIGURABLE(ANNRepresentation)

void ANNRepresentation::request(const std::string &role, ConfigurationRequest *config)
{
  if (role == "action")
  {
    config->push_back(CRP("inputs", "int.observation_dims", "Number of input dimensions", (int)inputs_, CRP::System, 1, MAX_NODES));
    config->push_back(CRP("output_min", "vector.action_min", "Lower limit on outputs", output_min_, CRP::System));
    config->push_back(CRP("output_max", "vector.action_max", "Upper limit on outputs", output_max_, CRP::System));
  }
  else
  {
    if (role == "transition" || role == "value/action")
      config->push_back(CRP("inputs", "int.observation_dims+int.action_dims", "Number of input dimensions", (int)inputs_, CRP::System, 1));
    else if (role == "value/state")
      config->push_back(CRP("inputs", "int.observation_dims", "Number of input dimensions", (int)inputs_, CRP::System, 1));
    else
      config->push_back(CRP("inputs", "Number of input dimensions", (int)inputs_, CRP::System, 1));
    
    config->push_back(CRP("output_min", "Lower limit on outputs", output_min_, CRP::System));
    config->push_back(CRP("output_max", "Upper limit on outputs", output_max_, CRP::System));
  }
  
  config->push_back(CRP("hiddens", "Number of hidden nodes", (int)hiddens_, CRP::Configuration, 0, MAX_NODES));

  config->push_back(CRP("steepness", "Steepness of activation function", steepness_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("bias", "Use bias nodes", bias_, CRP::Configuration, 0, 1));
  config->push_back(CRP("recurrent", "Feed hidden activation back as input", recurrent_, CRP::Configuration, 0, 1));
}

void ANNRepresentation::configure(Configuration &config)
{
  inputs_ = config["inputs"];
  output_min_ = config["output_min"];
  output_max_ = config["output_max"];
  
  outputs_ = output_min_.size();
  hiddens_ = config["hiddens"];
  if (!hiddens_)
    hiddens_ = ceil((inputs_+outputs_)/2.);
    
  if (output_min_.size() != output_max_.size() || outputs_ > MAX_NODES)
    throw bad_param("representation/parameterized/ann:{output_min,output_max}");
    
  state_.resize(hiddens_);
  weights_.resize(size());
    
  steepness_ = config["steepness"];
  bias_ = config["bias"];
  recurrent_ = config["recurrent"];
 
  INFO("Structure [" << inputs_ << ", " << hiddens_ << ", " << outputs_ << "] " << (bias_?"with":"without") << " bias (" << size() << " parameters), steepness " << steepness_);
  
  // Initialize memory
  reset();
}

void ANNRepresentation::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    for (size_t ii=0; ii < hiddens_; ++ii)
      state_[ii] = 0.;
      
    // Initialize memory
    for (size_t ii=0; ii < size(); ++ii)
      weights_[ii] = RandGen::getUniform(-1, 1);
  }
}

ANNRepresentation *ANNRepresentation::clone() const
{
  return new ANNRepresentation(*this);
}

double ANNRepresentation::read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const
{
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  
  if (vp)
  {
    double hidden[MAX_NODES], output[MAX_NODES];
    size_t hidden_params = inputs_+bias_+recurrent_;
    
    // Calculate hidden activation
    for (size_t hh=0; hh < hiddens_; ++hh)
    {
      const double *w = &weights_[hidden_params*hh];
      double act = 0;
      if (bias_)      act += w[inputs_];                  // Bias
      if (recurrent_) act += state_[hh]*w[inputs_+bias_]; // Recurrence
      
      for (size_t ii=0; ii < inputs_; ++ii)
        act += w[ii]*vp->vector[ii];
        
      hidden[hh] = activate(act);
    }
    
    // TODO: Remember state
    // memcpy(state_.data(), hidden, hiddens_*sizeof(double));
    
    // Calculate output activation
    for (size_t oo=0; oo < outputs_; ++oo)
    {
      const double *w = &weights_[hidden_params*hiddens_+(hiddens_+bias_)*oo];
      double act = 0;
      if (bias_) act += w[hiddens_]; // Bias
      
      for (size_t hh=0; hh < hiddens_; ++hh)
        act += w[hh]*hidden[hh];
        
      output[oo] = activate(act);
    }
    
    // Normalize outputs from 0..1 to output_min_..output_max_
    result->resize(outputs_);
    for (size_t oo=0; oo < outputs_; ++oo)
      (*result)[oo] = output_min_[oo]+output[oo]*(output_max_[oo]-output_min_[oo]);
  }
  else
    throw Exception("representation/parameterized/ann requires a projector returning a VectorProjection");
    
  if (stddev) stddev->clear();
  
  return (*result)[0];
}

void ANNRepresentation::write(const ProjectionPtr projection, const Vector &target, const Vector &alpha)
{
  // TODO
}

void ANNRepresentation::update(const ProjectionPtr projection, const Vector &delta)
{
  // TODO
}
