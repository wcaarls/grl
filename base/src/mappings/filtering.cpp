/** \file filtering.cpp
 * \brief Filtering mapping source file. 
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-07-18 
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

#include <grl/mappings/filtering.h>

using namespace grl;

REGISTER_CONFIGURABLE(FilteringMapping)
REGISTER_CONFIGURABLE(FilteringPolicy)

// Generate new vector newv[ii] = v[index[ii]], where v[-1] = 0.
static Vector reindex(const Vector &v, const Vector &index)
{
  Vector newv = ConstantVector(index.size(), 0.);
  
  for (size_t ii=0; ii < index.size(); ++ii)
  {
    int idx = round(index[ii]);
  
    if (idx >= 0)
    {
      if (idx >= v.size())
      {
        ERROR("Index vector " << index << " incompatible with source vector " << v);
        throw bad_param("mapping/filtering:{input_idx,output_idx}");
      }
    
      newv[ii] = v[idx];
    }
  }
  
  return newv;
}

void FilteringMapping::request(ConfigurationRequest *config)
{
  config->push_back(CRP("input_idx", "vector", "Index vector for downstream inputs (-1=pad)", input_idx_));
  config->push_back(CRP("output_idx", "vector", "Index vector for upstream outputs (-1=pad)", output_idx_));

  config->push_back(CRP("mapping", "mapping", "Downstream mapping", mapping_));
}

void FilteringMapping::configure(Configuration &config)
{
  input_idx_ = config["input_idx"].v();
  output_idx_ = config["output_idx"].v();

  mapping_ = (Policy*)config["mapping"].ptr();
}

void FilteringMapping::reconfigure(const Configuration &config)
{
}

double FilteringMapping::read(const Vector &in, Vector *result) const
{
  Vector res;

  mapping_->read(reindex(in, input_idx_), &res);
  *result = reindex(res, output_idx_);
  
  return (*result)[0];
}

void FilteringMapping::read(const Matrix &in, Matrix *result) const
{
  Matrix inremap = Matrix(in.rows(), input_idx_.size());
  for (size_t ii=0; ii != in.rows(); ++ii)
    inremap.row(ii) = reindex(in.row(ii), input_idx_);
    
  Matrix res;
  mapping_->read(inremap, &res);
  
  *result = Matrix(res.rows(), output_idx_.size());
  for (size_t ii=0; ii != res.rows(); ++ii)
    result->row(ii) = reindex(res.row(ii), output_idx_);
}

void FilteringPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("observation_idx", "vector", "Index vector for downstream observations (-1=pad)", observation_idx_));
  config->push_back(CRP("action_idx", "vector", "Index vector for upstream actions (-1=pad)", action_idx_));

  config->push_back(CRP("policy", "mapping/policy", "Downstream policy", policy_));
}

void FilteringPolicy::configure(Configuration &config)
{
  observation_idx_ = config["observation_idx"].v();
  action_idx_ = config["action_idx"].v();

  policy_ = (Policy*)config["policy"].ptr();
}

void FilteringPolicy::reconfigure(const Configuration &config)
{
}

void FilteringPolicy::act(double time, const Observation &in, Action *out)
{
  Action res;
  
  // TODO: should do inverse mapping of out->res here.

  policy_->act(time, reindex(in, observation_idx_), &res);
  out->type = res.type;
  out->v = reindex(res.v, action_idx_);
  out->logp = res.logp;
}

void FilteringPolicy::act(const Observation &in, Action *out) const
{
  Action res;

  // TODO: should do inverse mapping of out->res here.

  policy_->act(reindex(in, observation_idx_), &res);
  out->type = res.type;
  out->v = reindex(res.v, action_idx_);
  out->logp = res.logp;
}
