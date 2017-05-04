/** \file rbf.cpp
 * \brief Triangular RBF projector source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-11-29
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

#include <grl/projectors/rbf.h>

#define EPS 0.00001

using namespace grl;

REGISTER_CONFIGURABLE(TriangleRBFProjector)
REGISTER_CONFIGURABLE(GaussianRBFProjector)

void RBFProjector::request(const std::string &role, ConfigurationRequest *config)
{
  if (role == "observation")
  {
    config->push_back(CRP("input_min", "vector.observation_min", "Lower input dimension limit", min_, CRP::System));
    config->push_back(CRP("input_max", "vector.observation_max", "Upper input dimension limit", max_, CRP::System));
  }
  else if (role == "action")
  {
    config->push_back(CRP("input_min", "vector.action_min", "Lower input dimension limit", min_, CRP::System));
    config->push_back(CRP("input_max", "vector.action_max", "Upper input dimension limit", max_, CRP::System));
  }
  else if (role == "pair")
  {
    config->push_back(CRP("input_min", "vector.observation_min+vector.action_min", "Lower input dimension limit", min_, CRP::System));
    config->push_back(CRP("input_max", "vector.observation_max+vector.action_max", "Upper input dimension limit", max_, CRP::System));
  }
  else
  {
    config->push_back(CRP("input_min", "Lower input dimension limit", min_, CRP::System));
    config->push_back(CRP("input_max", "Upper input dimension limit", max_, CRP::System));
  }

  config->push_back(CRP("steps", "Basis functions per dimension", steps_, CRP::Configuration));
  config->push_back(CRP("memory", "int.memory", "Feature vector size", CRP::Provided));
}

void RBFProjector::configure(Configuration &config)
{
  min_ = config["input_min"].v();
  if (!min_.size())
    throw bad_param("projector/rbf:min");

  max_ = config["input_max"].v();
  if (!max_.size())
    throw bad_param("projector/rbf:max");

  steps_ = config["steps"].v();
  if (!steps_.size())
    throw bad_param("projector/rbf:steps");

  if (min_.size() != max_.size() || min_.size() != steps_.size())
    throw bad_param("projector/rbf:{min,max,steps}");

  delta_ = (max_-min_)/(steps_-1);

  // Cache strides per dimension
  stride_ = IndexVector(steps_.size());
  size_t ff = 1;
  for (size_t dd=0; dd < steps_.size(); ++dd)
  {
    stride_[dd] = ff;
    ff *= steps_[dd];
  }
  
  config.set("memory", (int)prod(steps_));
}

void RBFProjector::reconfigure(const Configuration &config)
{
}

ProjectionPtr TriangleRBFProjector::project(const Vector &in) const
{
  if (in.size() != min_.size())
    throw bad_param("projector/rbf/triangle:{min,max,steps}");
    
  // Number of activated basis functions
  size_t acts = pow(2, steps_.size());

  IndexProjection *p = new IndexProjection();
  p->indices.resize(acts);
  p->weights.resize(acts);
  
  // Find base grid cell
  IndexVector v(steps_.size());
  Vector dist(steps_.size());
  size_t index = 0;
  for (size_t dd=0; dd < steps_.size(); ++dd)
  {
    // Position in continuous grid coordinates
    double gridpos = std::min(std::max((in[dd]-min_[dd])/delta_[dd], 0.), steps_[dd]-1-1e-10);

    // Discretize  
    v[dd] = gridpos;
    index += v[dd]*stride_[dd];
    
    // Relative distance to left edge of cell
    dist[dd] = gridpos - v[dd];
  }
    
  // Iterate over activated basis functions
  for (size_t ii=0; ii < acts; ++ii)
  {
    size_t diff = 0;
    double w = 1;
    for (size_t dd=0; dd < steps_.size(); ++dd)
    {
      // Determine whether we're evaluating RBF on left (0) or right (1) edge of grid cell
      bool edge = (ii&(1<<dd))>0;
    
      // Keep track of difference in index
      diff += edge*stride_[dd];
      
      // Fuzzy AND using multiplication
      w *= edge?dist[dd]:(1-dist[dd]);
    }
    
    p->indices[ii] = index + diff;
    p->weights[ii] = w;
  }
  
  return ProjectionPtr(p);
}

void GaussianRBFProjector::request(const std::string &role, ConfigurationRequest *config)
{
  RBFProjector::request(role, config);

  config->push_back(CRP("sigma", "Standard deviation normalized to rbf spacing", sigma_, CRP::Configuration));
  config->push_back(CRP("cutoff", "Activation cutoff", cutoff_, CRP::Configuration));
}

void GaussianRBFProjector::configure(Configuration &config)
{
  RBFProjector::configure(config);
  
  sigma_ = config["sigma"];
  cutoff_ = config["cutoff"];
}

void GaussianRBFProjector::reconfigure(const Configuration &config)
{
  RBFProjector::reconfigure(config);
}

ProjectionPtr GaussianRBFProjector::project(const Vector &in) const
{
  if (in.size() != min_.size())
    throw bad_param("projector/rbf/gaussian:{min,max,steps}");

  int rbfs = prod(steps_);
  double sigmasq = sigma_*sigma_;
  double cutoff_distsq = -sigmasq*std::log(cutoff_);

  IndexProjection *p = new IndexProjection;
  p->indices.reserve(rbfs);
  p->weights.reserve(rbfs);
  
  // Iterate over all basis functions
  Vector inmin = (min_-in)/delta_, inmax = (max_-in)/delta_;
  Vector diff = inmin;
  for (int ii=0; ii < rbfs; ++ii)
  {
    double distsq = dot(diff, diff);
    
    if (distsq < cutoff_distsq)
    {
      p->indices.push_back(ii);
      p->weights.push_back(exp(-distsq/sigmasq));
    }
            
    for (int dd=0; dd < steps_.size(); ++dd)
    {
      diff[dd] = diff[dd] + 1;
      
      if (diff[dd] > (inmax[dd]+EPS))
        diff[dd] = inmin[dd];
      else
        break;
    }
  }
  
  return ProjectionPtr(p);
}
