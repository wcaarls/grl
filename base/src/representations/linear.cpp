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

#include <algorithm>
#include <grl/representations/linear.h>

using namespace grl;

REGISTER_CONFIGURABLE(LinearRepresentation)

void LinearRepresentation::request(const std::string &role, ConfigurationRequest *config)
{
  config->push_back(CRP("init_min", "Lower initial value limit", init_min_));
  config->push_back(CRP("init_max", "Upper initial value limit", init_max_));

  config->push_back(CRP("memory", "int.memory", "Feature vector size", (int)memory_, CRP::System));

  if (role == "action")
  {
    config->push_back(CRP("outputs", "int.action_dims", "Number of outputs", (int)outputs_, CRP::System));
    config->push_back(CRP("output_min", "vector.action_min", "Lower output limit", output_min_, CRP::System));
    config->push_back(CRP("output_max", "vector.action_max", "Upper output limit", output_max_, CRP::System));
  }
  else if (role == "transition")
  {
    config->push_back(CRP("outputs", "int.observation_dims+2", "Number of outputs", (int)outputs_, CRP::System));
    config->push_back(CRP("output_min", "Lower output limit", output_min_, CRP::System));
    config->push_back(CRP("output_max", "Upper output limit", output_max_, CRP::System));
  }
  else
  {
    config->push_back(CRP("outputs", "Number of outputs", (int)outputs_, CRP::System));
    config->push_back(CRP("output_min", "Lower output limit", output_min_, CRP::System));
    config->push_back(CRP("output_max", "Upper output limit", output_max_, CRP::System));
  }
}

void LinearRepresentation::configure(Configuration &config)
{
  memory_ = config["memory"];
  outputs_ = config["outputs"];
  
  init_min_ = config["init_min"].v();
  if (init_min_.size() && init_min_.size() < outputs_)
    init_min_ = ConstantVector(outputs_, init_min_[0]);
  if (init_min_.size() != outputs_)
    throw bad_param("representation/parameterized/linear:init_min");
  
  init_max_ = config["init_max"].v();
  if (init_max_.size() && init_max_.size() < outputs_)
    init_max_ = ConstantVector(outputs_, init_max_[0]);
  if (init_max_.size() != outputs_)
    throw bad_param("representation/parameterized/linear:max");

  params_.resize(memory_ * outputs_);
  
  output_min_ = config["output_min"].v();
  if (!output_min_.size())
    output_min_ = ConstantVector(outputs_, -DBL_MAX);
  if (output_min_.size() != outputs_)
    throw bad_param("representation/parameterized/linear:output_min");
    
  output_max_ = config["output_max"].v();
  if (!output_max_.size())
    output_max_ = ConstantVector(outputs_, DBL_MAX);
  if (output_max_.size() != outputs_)
    throw bad_param("representation/parameterized/linear:output_max");
  
  // Initialize memory
  reset();
}

void LinearRepresentation::reconfigure(const Configuration &config)
{
  if (config.has("action"))
  {
    if (config["action"].str() == "reset")
    {
      TRACE("Initializing " << memory_ << " values between " << init_min_ << " and " << init_max_);
    
      params_.resize(memory_ * outputs_);

      // Initialize memory
      Rand *rand = RandGen::instance();
      for (size_t ii=0; ii < memory_; ++ii)
        for (size_t jj=0; jj < outputs_; ++jj)  
          params_[ii*outputs_+jj] = rand->getUniform(init_min_[jj], init_max_[jj]);
    }
    else if (config["action"].str() == "load")
    {
      std::string cfg_path = path();
      std::replace(cfg_path.begin(), cfg_path.end(), '/', '_');
      std::string file = config["file"].str() + cfg_path + ".dat";

      FILE *f = fopen(file.c_str(), "rb");
      if (!f)
      {
        WARNING("Could not open '" << file << "' for reading");
        return;
      }
      
      fseek(f, 0, SEEK_END);
      if (ftell(f) != (long int)(params_.size() * sizeof(double)))
      {
        WARNING("Configuration mismatch for '" << file << "'");
        fclose(f);
        return;
      }
      
      fseek(f, 0, SEEK_SET);
      if (fread(params_.data(), sizeof(double), params_.size(), f) != params_.size())
      {
        WARNING("Could not read '" << file << "'");
        fclose(f);
        return;
      }
      fclose(f);
    }
    else if (config["action"].str() == "save")
    {
      std::string cfg_path = path();
      std::replace(cfg_path.begin(), cfg_path.end(), '/', '_');
      std::string file = config["file"].str() + cfg_path + ".dat";

      FILE *f = fopen(file.c_str(), "wb");
      if (!f)
      {
        WARNING("Could not open '" << file << "' for writing");
        return;
      }
      
      fwrite(params_.data(), sizeof(double), params_.size(), f);
      fclose(f);
    }
  }  
}

LinearRepresentation &LinearRepresentation::copy(const Configurable &obj)
{
  const LinearRepresentation &lr = dynamic_cast<const LinearRepresentation&>(obj);
  
  params_ = lr.params_;

  return *this;
}

double LinearRepresentation::read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const
{
  Projection &p = *projection;
  
  IndexProjection *ip = dynamic_cast<IndexProjection*>(&p);
  if (ip)
  {
    *result = ConstantVector(outputs_, 0);
    
    if (ip->weights.empty())
    {
      for (size_t ii=0; ii < ip->indices.size(); ++ii)
        for (size_t jj=0; jj < outputs_; ++jj)
          (*result)[jj] += params_[ip->indices[ii]*outputs_+jj];
      for (size_t jj=0; jj < outputs_; ++jj)
        (*result)[jj] /= ip->indices.size();
    }
    else
    {
      for (size_t ii=0; ii < ip->indices.size(); ++ii)
        for (size_t jj=0; jj < outputs_; ++jj)
          (*result)[jj] += params_[ip->indices[ii]*outputs_+jj]*ip->weights[ii];
    }
  }
  else
  {
    VectorProjection *vp = dynamic_cast<VectorProjection*>(&p);
    if (vp)
    {
      if (vp->vector.size() != memory_)
        throw bad_param("representation/parameterized/linear:memory (or matching projector)");
    
      *result = ConstantVector(outputs_, 0);
      
      for (size_t ii=0; ii < vp->vector.size(); ++ii)
        for (size_t jj=0; jj < outputs_; ++jj)
          (*result)[jj] += params_[ii*outputs_+jj]*vp->vector[ii];
    }
    else
      throw Exception("representation/parameterized/linear requires a projector returning IndexProjection or VectorProjection");
  }

  for (size_t ii=0; ii < outputs_; ++ii)
    (*result)[ii] = fmin(fmax((*result)[ii], output_min_[ii]), output_max_[ii]);

  if (stddev) *stddev = Vector();
  
  return (*result)[0];
}

void LinearRepresentation::write(const ProjectionPtr projection, const Vector &target, const Vector &alpha)
{
  if (target.size() != alpha.size())
    throw Exception("Learning rate vector does not match target vector");

  // TODO: Store read values and update those (for thread safety)
  Vector value;
  read(projection, &value, NULL);
  Vector delta = alpha*(target-value);
  update(projection, delta);
}

void LinearRepresentation::update(const ProjectionPtr projection, const Vector &delta)
{
  Projection &p = *projection;
  
  IndexProjection *ip = dynamic_cast<IndexProjection*>(&p);
  if (ip)
  {
    if (ip->weights.empty())
    {  
      for (size_t ii=0; ii != ip->indices.size(); ++ii)
        for (size_t jj=0; jj < outputs_; ++jj)
          if (ip->indices[ii] != IndexProjection::invalid_index())
            params_[ip->indices[ii]*outputs_+jj] = fmin(fmax(params_[ip->indices[ii]*outputs_+jj] + delta[jj], output_min_[jj]), output_max_[jj]);
    }
    else
    {
      // Calculate step size for one-step learning
      double norm2 = 0;
      for (size_t ii=0; ii != ip->weights.size(); ++ii)
        norm2 += ip->weights[ii]*ip->weights[ii];

      // Avoid division by zero
      if (norm2 < 0.001)
        norm2 = 0.001;

      for (size_t ii=0; ii != ip->indices.size(); ++ii)
        for (size_t jj=0; jj < outputs_; ++jj)
          if (ip->indices[ii] != IndexProjection::invalid_index())
            params_[ip->indices[ii]*outputs_+jj] = fmin(fmax(params_[ip->indices[ii]*outputs_+jj] + ip->weights[ii]*delta[jj]/norm2, output_min_[jj]), output_max_[jj]);
    }
  }
  else
  {
    VectorProjection *vp = dynamic_cast<VectorProjection*>(&p);
    if (vp)
    {
      if (vp->vector.size() != memory_)
        throw bad_param("representation/parameterized/linear:memory (or matching projector)");

      // Calculate step size for one-step learning
      double norm2 = 0;
      for (size_t ii=0; ii != vp->vector.size(); ++ii)
        norm2 += vp->vector[ii]*vp->vector[ii];

      // Avoid division by zero
      if (norm2 < 0.001)
        norm2 = 0.001;

      for (size_t ii=0; ii != vp->vector.size(); ++ii)
        for (size_t jj=0; jj < outputs_; ++jj)
          params_[ii*outputs_+jj] = fmin(fmax(params_[ii*outputs_+jj] + vp->vector[ii]*delta[jj]/norm2, output_min_[jj]), output_max_[jj]);
    }
    else
      throw Exception("representation/parameterized/linear requires a projector returning IndexProjection or VectorProjection");
  }
}
