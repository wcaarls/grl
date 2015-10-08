/** \file bounded_q.cpp
 * \brief Bounded Q policy source file. 
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-11
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

#include <grl/policies/bounded_q.h>

using namespace grl;

REGISTER_CONFIGURABLE(BoundedQPolicy)

void BoundedQPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("bound", "Maximum action delta", bound_));
  QPolicy::request(config);
}

void BoundedQPolicy::configure(Configuration &config)
{
  QPolicy::configure(config);
  
  bound_ = config["bound"];
  
  if (bound_.size() != variants_[0].size())
    throw bad_param("policy/discrete/q/bounded:bound");
}

void BoundedQPolicy::reconfigure(const Configuration &config)
{
}

BoundedQPolicy *BoundedQPolicy::clone() const
{
  BoundedQPolicy *qp = new BoundedQPolicy(*this);
  qp->discretizer_ = discretizer_->clone();
  qp->projector_ = projector_->clone();
  qp->representation_ = representation_->clone();
  qp->sampler_ = sampler_->clone();
  
  return qp;
}

void BoundedQPolicy::act(double time, const Vector &in, Vector *out)
{
  if (out->size())
  {
    Vector qvalues, filtered;
    std::vector<size_t> idx;
    
    values(in, &qvalues);
    filter(*out, qvalues, &filtered, &idx);
    
    size_t action = sampler_->sample(filtered);
    *out = variants_[idx[action]];
  }
  else
    QPolicy::act(in, out); 
}

/**
 * Returns both the Q values of the valid actions, and
 * an index array such that filtered[ii] = qvalues[idx[ii]]
 */
void BoundedQPolicy::filter(const Vector &prev_out, const Vector &qvalues, Vector *filtered, std::vector<size_t> *idx) const
{
  if (prev_out.size() != bound_.size())
    ERROR("Previous action has wrong size");
    
  idx->clear();
  idx->reserve(qvalues.size());
  
  for (size_t ii=0; ii < variants_.size(); ++ii)
  {
    bool valid=true;
    for (size_t jj=0; jj < prev_out.size(); ++jj)
    {
      if (fabs(variants_[ii][jj] - prev_out[jj]) > bound_[jj])
      {
        valid=false;
        break;
      }
    }
    
    if (valid)
      idx->push_back(ii);
  }
  
  filtered->resize(idx->size());
  for (size_t ii=0; ii < idx->size(); ++ii)
    (*filtered)[ii] = qvalues[(*idx)[ii]];
}
            