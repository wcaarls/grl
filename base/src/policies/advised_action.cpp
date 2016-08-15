/** \file advised_action.cpp
 * \brief Advised Action policy source file.
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

#include <grl/policies/advised_action.h>

using namespace grl;

REGISTER_CONFIGURABLE(AdvisedActionPolicy)
/*
void BoundedActionPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("bound", "Maximum action delta", bound_));
  ActionPolicy::request(config);
}

void BoundedActionPolicy::configure(Configuration &config)
{
  ActionPolicy::configure(config);
  
  bound_ = config["bound"].v();
  
  if (bound_.size() != variants_[0].size())
    throw bad_param("policy/discrete/q/bounded:bound");
}

void BoundedActionPolicy::reconfigure(const Configuration &config)
{
}
*/

AdvisedActionPolicy *AdvisedActionPolicy::clone() const
{
  AdvisedActionPolicy *ap = new AdvisedActionPolicy(*this);
  ap->projector_ = projector_->clone();
  ap->representation_ = representation_->clone();
  return ap;
}

TransitionType AdvisedActionPolicy::act(const Vector &in, Vector *out) const
{
  Vector process_out;
  process_out.resize(out->size());

  ProjectionPtr p = projector_->project(in);
  representation_->read(p, &process_out);

  for (size_t ii=0; ii < out->size(); ++ii)
  {
    if (sigma_[ii])
      process_out[ii] += RandGen::getNormal(0., sigma_[ii]);

    (*out)[ii] = fmin(fmax(process_out[ii] + (*out)[ii], min_[ii]), max_[ii]);
  }

  // Some representations may not always return a value.
  if (!out->size())
  {
    *out = (min_+max_)/2;
    return ttGreedy;
  }
  else
    return ttExploratory;
}

