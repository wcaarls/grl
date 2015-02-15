/** \file parameterized.cpp
 * \brief Continuous and discrete parameterized policies source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-13
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

#include <grl/policies/parameterized.h>

using namespace grl;

REGISTER_CONFIGURABLE(ActionPolicy)
REGISTER_CONFIGURABLE(ProbabilityPolicy)

void ActionPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("min", "Lower action limit", min_));
  config->push_back(CRP("max", "Upper action limit", max_));

  config->push_back(CRP("projector", "projector", "Projects observations onto representation space", projector_));
  config->push_back(CRP("representation", "representation/parameterized", "Action representation", representation_));
}

void ActionPolicy::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (ParameterizedRepresentation*)config["representation"].ptr();
  
  min_ = config["min"];
  max_ = config["max"];
  
  if (min_.size() != max_.size())
    throw bad_param("policy/parameterized/action:{min,max}");
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
  
  if (!min_.empty())
  {
    if (out->size() != min_.size())
      throw bad_param("policy/parameterized/action:{min,max}");
    
    for (size_t ii=0; ii < out->size(); ++ii)
      (*out)[ii] = fmin(fmax((*out)[ii], min_[ii]), max_[ii]);
  }
}

void ProbabilityPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("discretizer", "discretizer", "Action discretizer", discretizer_));
  config->push_back(CRP("projector", "projector", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation/parameterized", "Probability representation", representation_));
}

void ProbabilityPolicy::configure(Configuration &config)
{
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  discretizer_->options(&variants_);
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (ParameterizedRepresentation*)config["representation"].ptr();
}

void ProbabilityPolicy::reconfigure(const Configuration &config)
{
}

ProbabilityPolicy *ProbabilityPolicy::clone() const
{
  ProbabilityPolicy *dpp = new ProbabilityPolicy(*this);
  dpp->discretizer_ = discretizer_->clone();
  dpp->projector_ = projector_->clone();
  dpp->representation_ = representation_->clone();
  return dpp;
}

void ProbabilityPolicy::act(const Vector &in, Vector *out) const
{
  std::vector<ProjectionPtr> projections;
  projector_->project(in, variants_, &projections);

  Vector dist(variants_.size()), v;
  
  for (size_t ii=0; ii < variants_.size(); ++ii)
    dist[ii] = representation_->read(projections[ii], &v);
    
  CRAWL(dist);
    
  *out = variants_[sample(dist)];
}
