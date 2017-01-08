/** \file ucb.cpp
 * \brief UCB policy source file. 
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-20
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

#include <grl/policies/ucb.h>

using namespace grl;

REGISTER_CONFIGURABLE(UCBPolicy)

void UCBPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("projector", "projector.pair", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "Q-value representation", representation_));
  config->push_back(CRP("visit_representation", "representation.value/action", "Visit count representation", visit_representation_));

  config->push_back(CRP("c_p", "UCB1 exploration term", c_p_, CRP::Online, 0., DBL_MAX));
}

void UCBPolicy::configure(Configuration &config)
{
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  visit_representation_ = (Representation*)config["visit_representation"].ptr();
  
  c_p_ = config["c_p"];
}

void UCBPolicy::reconfigure(const Configuration &config)
{
}

TransitionType UCBPolicy::act(const Vector &in, Vector *out) const
{
  std::vector<Vector> variants;
  discretizer_->options(in, &variants);

  LargeVector qvalues(variants.size()), visits(variants.size());
  double state_visits = 0;
  std::vector<ProjectionPtr> projections;
  projector_->project(in, variants, &projections);
  
  Vector value;
  
  for (size_t ii=0; ii < variants.size(); ++ii)
  {
    qvalues[ii] = representation_->read(projections[ii], &value);
    visits[ii] = visit_representation_->read(projections[ii], &value);
    state_visits += visits[ii];
  }
  
  state_visits = std::log(state_visits);
  
  size_t action = 0;
  for (size_t ii=0; ii < variants.size(); ++ii)
  {
    // UCB1 exploration term
    qvalues[ii] += 2*c_p_*sqrt(state_visits/fmax(1, visits[ii]));
  
    if (qvalues[ii] > qvalues[action])
      action = ii;
  }
  
  *out = variants[action];
  return ttGreedy;
}

TransitionType UCBPolicy::act(double time, const Vector &in, Vector *out)
{
  TransitionType tt = act(in, out);
  
  ProjectionPtr projection = projector_->project(in, *out);
  
  Vector v;
  double visits = visit_representation_->read(projection, &v);
  visit_representation_->write(projection, VectorConstructor(visits+1));
  visit_representation_->finalize();
  return tt;
}
