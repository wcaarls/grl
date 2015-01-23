/** \file linear.cpp
 * \brief Linear representation source file.
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

#include <grl/representations/linear.h>

using namespace grl;

REGISTER_CONFIGURABLE(LinearRepresentation)

void LinearRepresentation::request(ConfigurationRequest *config)
{
  config->push_back(CRP("memory", "Feature vector size", (int)memory_));
  config->push_back(CRP("outputs", "Number of outputs", (int)outputs_));

  config->push_back(CRP("min", "Lower initial value limit", min_));
  config->push_back(CRP("max", "Upper initial value limit", max_));
}

void LinearRepresentation::configure(Configuration &config)
{
  memory_ = config["memory"];
  outputs_ = config["outputs"];
  
  min_ = config["min"];
  if (min_.size() && min_.size() < outputs_)
    min_.resize(outputs_, min_[0]);
  if (min_.size() != outputs_)
    throw bad_param("representation/linear:min");
  
  max_ = config["max"];
  if (max_.size() && max_.size() < outputs_)
    max_.resize(outputs_, max_[0]);
  if (max_.size() != outputs_)
    throw bad_param("representation/linear:max");

  params_.resize(memory_ * outputs_);
  
  // Initialize memory
  Rand *rand = RandGen::instance();
  for (size_t ii=0; ii < memory_; ++ii)
    for (size_t jj=0; jj < outputs_; ++jj)
      params_[ii*outputs_+jj] = rand->getUniform(min_[jj], max_[jj]);
}

void LinearRepresentation::reconfigure(const Configuration &config)
{
}

LinearRepresentation *LinearRepresentation::clone() const
{
  LinearRepresentation *lr = new LinearRepresentation();
  lr->params_ = params_;
  return lr;
}

double LinearRepresentation::read(const ProjectionPtr &projection, Vector *result) const
{
  Projection &p = *projection;
  
  IndexProjection *ip = dynamic_cast<IndexProjection*>(&p);
  if (ip)
  {
    result->clear();
    result->resize(outputs_, 0);
    
    for (size_t ii=0; ii < ip->indices.size(); ++ii)
      for (size_t jj=0; jj < outputs_; ++jj)
        (*result)[jj] += params_[ip->indices[ii]*outputs_+jj];
    for (size_t jj=0; jj < outputs_; ++jj)
      (*result)[jj] /= ip->indices.size();
  }
  else
  {
    VectorProjection *vp = dynamic_cast<VectorProjection*>(&p);
    if (vp)
    {
      result->clear();
      result->resize(outputs_, 0);
      
      double activation=0;
      for (size_t ii=0; ii < vp->vector.size(); ++ii)
      {
        activation += vp->vector[ii];
        for (size_t jj=0; jj < outputs_; ++jj)
          (*result)[jj] += params_[ii*outputs_+jj]*vp->vector[ii];
      }
      for (size_t jj=0; jj < outputs_; ++jj)
        (*result)[jj] /= activation;
    }
    else
      throw Exception("Unknown projection for LinearRepresentation");
  }
  
  return (*result)[0];
}

void LinearRepresentation::write(const ProjectionPtr projection, const Vector &target, double alpha)
{
  // TODO: Store read values and update those (for thread safety)
  Vector value;
  read(projection, &value);
  Vector delta = alpha*(target-value);
  
  update(projection, delta);
}

void LinearRepresentation::update(const ProjectionPtr projection, const Vector &delta)
{
  Projection &p = *projection;
  
  IndexProjection *ip = dynamic_cast<IndexProjection*>(&p);
  if (ip)
  {
    for (size_t ii=0; ii != ip->indices.size(); ++ii)
      for (size_t jj=0; jj < outputs_; ++jj)
        if (ip->indices[ii] != IndexProjection::invalid_index())
          params_[ip->indices[ii]*outputs_+jj] += delta[jj];
  }
  else
  {
    VectorProjection *vp = dynamic_cast<VectorProjection*>(&p);
    if (vp)
    {
      for (size_t ii=0; ii != vp->vector.size(); ++ii)
        for (size_t jj=0; jj < outputs_; ++jj)
          params_[ii*outputs_+jj] += vp->vector[ii]*delta[jj];
    }
    else
      throw Exception("Unknown projection for LinearRepresentation");
  }
}
