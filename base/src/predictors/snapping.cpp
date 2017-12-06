/** \file snapping.cpp
 * \brief Predictor that snaps to grid centers source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-12-05
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

#include <grl/predictors/snapping.h>

using namespace grl;

REGISTER_CONFIGURABLE(SnappingPredictor)

void SnappingPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);

  config->push_back(CRP("input_min", "vector.observation_min", "Lower observation dimension limit", min_, CRP::System));
  config->push_back(CRP("input_max", "vector.observation_max", "Upper observation dimension limit", max_, CRP::System));
  config->push_back(CRP("steps", "Centers per observation dimension", steps_, CRP::Configuration));
  
  config->push_back(CRP("centers", "Number of closest centers to snap to (0=all)", (int)centers_));
  
  config->push_back(CRP("model", "observation_model", "Observation model used for planning", model_));
  config->push_back(CRP("predictor", "predictor", "Downstream predictor", predictor_));
}

void SnappingPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  min_ = config["input_min"].v();
  if (!min_.size())
    throw bad_param("projector/snapping:min");

  max_ = config["input_max"].v();
  if (!max_.size())
    throw bad_param("projector/snapping:max");

  steps_ = config["steps"].v();
  if (!steps_.size())
    throw bad_param("projector/snapping:steps");

  if (min_.size() != max_.size() || min_.size() != steps_.size())
    throw bad_param("projector/snapping:{min,max,steps}");

  delta_ = (max_-min_)/(steps_-1);

  // Cache strides per dimension
  stride_ = IndexVector(steps_.size());
  size_t ff = 1;
  for (size_t dd=0; dd < steps_.size(); ++dd)
  {
    stride_[dd] = ff;
    ff *= steps_[dd];
  }
  
  centers_ = config["centers"];
  if (!centers_)
    centers_ = pow(2, steps_.size());
  if (centers_ > pow(2, steps_.size()))
    throw bad_param("projector/snapping:centers");
  
  model_ = (ObservationModel*)config["model"].ptr();
  predictor_ = (Predictor*)config["predictor"].ptr();
}  

void SnappingPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);
}

void SnappingPredictor::update(const Transition &transition)
{
  Predictor::update(transition);
  
  if (transition.prev_obs.size() > min_.size())
    throw bad_param("projector/snapping:{min,max,steps}");
    
  // Number of activated basis functions
  size_t acts = pow(2, steps_.size());
  
  std::vector<Vector> centers(acts);
  Vector distances(acts);
  
  // Find base grid cell
  IndexVector v(steps_.size());
  Vector dist(steps_.size());
  for (size_t dd=0; dd < steps_.size(); ++dd)
  {
    // Position in continuous grid coordinates
    double gridpos = std::min(std::max((transition.prev_obs[dd]-min_[dd])/delta_[dd], 0.), steps_[dd]-1-1e-10);

    // Discretize  
    v[dd] = gridpos;
    
    // Relative distance to left edge of cell
    dist[dd] = gridpos - v[dd];
  }
  
  // Iterate over closest centers
  double maxdist = 0;
  for (size_t ii=0; ii < acts; ++ii)
  {
    centers[ii] = Vector(steps_.size());
  
    double w = 0;
    for (size_t dd=0; dd < steps_.size(); ++dd)
    {
      // Determine whether we're evaluating RBF on left (0) or right (1) edge of grid cell
      bool edge = (ii&(1<<dd))>0;
      
      // Calculate center
      centers[ii][dd] = min_[dd] + (v[dd] + edge)*delta_[dd];
    
      // L2 norm
      w += edge?(1-dist[dd])*(1-dist[dd]):dist[dd]*dist[dd];
    }
    
    distances[ii] = w;
    maxdist = fmax(maxdist, w);
  }
  
  // Convert distances into weights
  for (size_t ii=0; ii < acts; ++ii)
    distances[ii] = sqrt(exp(-distances[ii]/maxdist));
  
  IndexVector chosen = sample(distances, centers_);
  
  for (size_t ii=0; ii < centers_; ++ii)
  {
    Transition snapped = transition;
    snapped.prev_obs = centers[chosen[ii]];
    int terminal;
    
    model_->step(snapped.prev_obs, snapped.prev_action, &snapped.obs, &snapped.reward, &terminal);
    if (terminal == 2)
    {
      snapped.obs.absorbing = true;
      snapped.action = Action();
    }
    else
    {
      snapped.obs.absorbing = false;
      
      // NOTE: Should be policy action, but will be ignored because this predictor only
      // really makes sense for off-policy downstream predictors.
      snapped.action = snapped.prev_action;
    }
    
    predictor_->update(snapped);
  }
}
