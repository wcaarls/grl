/** \file observation.cpp
 * \brief Observation model source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-23
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

#include <grl/environments/observation.h>

using namespace grl;

REGISTER_CONFIGURABLE(FixedObservationModel)
REGISTER_CONFIGURABLE(ApproximatedObservationModel)
REGISTER_CONFIGURABLE(FixedRewardObservationModel)
 
// FixedObservationModel

void FixedObservationModel::request(ConfigurationRequest *config)
{
  config->push_back(CRP("model", "model", "Environment model", model_));
  config->push_back(CRP("task", "task", "Task to perform in the environment (should match model)", task_));
}

void FixedObservationModel::configure(Configuration &config)
{
  model_ = (Model*)config["model"].ptr();
  task_ = (Task*)config["task"].ptr();
}

void FixedObservationModel::reconfigure(const Configuration &config)
{
}

FixedObservationModel *FixedObservationModel::clone() const
{
  FixedObservationModel *om = new FixedObservationModel(*this);
  om->model_ = model_->clone();
  om->task_ = task_->clone();
  return om;
}

void FixedObservationModel::step(const Vector &obs, const Vector &action, Vector *next, double *reward, int *terminal) const
{
  Vector state, next_state;
  
  if (!task_->invert(obs, &state))
  {
    ERROR("Task does not support inversion");
    next->clear();
    return;
  }
  
  model_->step(state, action, &next_state);
  task_->observe(next_state, next, terminal);
  task_->evaluate(state, action, next_state, reward);
}

// ApproximatedObservationModel

void ApproximatedObservationModel::request(ConfigurationRequest *config)
{
  config->push_back(CRP("wrapping", "Wrapping boundaries", wrapping_));
  config->push_back(CRP("observation_min", "Lower limit on observations", observation_min_, CRP::System));
  config->push_back(CRP("observation_max", "Upper limit on observations", observation_max_, CRP::System));

  config->push_back(CRP("projector", "projector", "Projector for transition model (|S|+|A| dimensions)", projector_));
  config->push_back(CRP("representation", "representation", "Representation for transition model (|S|+2 dimensions)", representation_));
}

void ApproximatedObservationModel::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();

  wrapping_ = config["wrapping"];
  
  observation_min_ = config["observation_min"];
  observation_max_ = config["observation_max"];
}

void ApproximatedObservationModel::reconfigure(const Configuration &config)
{
}

ApproximatedObservationModel *ApproximatedObservationModel::clone() const
{
  ApproximatedObservationModel *om = new ApproximatedObservationModel(*this);
  om->projector_ = projector_->clone();
  om->representation_ = representation_->clone();
  return om;
}

void ApproximatedObservationModel::step(const Vector &obs, const Vector &action, Vector *next, double *reward, int *terminal) const
{
  ProjectionPtr p = projector_->project(extend(obs, action)); 
  
  if (!p)
  {
    next->clear();
    return;
  }
 
  representation_->read(p, next);
  
  bool valid = !next->empty();
  for (size_t ii=0; ii < obs.size() && valid; ++ii)
  {
    (*next)[ii] += obs[ii];
    
    if (wrapping_[ii])
      (*next)[ii] = fmod(fmod((*next)[ii], wrapping_[ii]) + wrapping_[ii], wrapping_[ii]);
    
    if ((*next)[ii] < observation_min_[ii] || (*next)[ii] > observation_max_[ii])
      valid = false;
  }

  // Guard against failed model prediction
  if (valid)
  {
    *reward = (*next)[next->size()-2];  
    *terminal = (int)(*next)[next->size()-1];
    next->resize(next->size()-2);
  }
  else
    next->clear();
}

// FixedRewardObservationModel

void FixedRewardObservationModel::request(ConfigurationRequest *config)
{
  ApproximatedObservationModel::request(config);
  
  config->push_back(CRP("task", "task", "Task to perform in the environment", task_));
}

void FixedRewardObservationModel::configure(Configuration &config)
{
  ApproximatedObservationModel::configure(config);
  
  task_ = (Task*)config["task"].ptr();
}

void FixedRewardObservationModel::reconfigure(const Configuration &config)
{
}

FixedRewardObservationModel *FixedRewardObservationModel::clone() const
{
  FixedRewardObservationModel *om = new FixedRewardObservationModel(*this);
  om->projector_ = projector_->clone();
  om->representation_ = representation_->clone();
  om->task_ = task_->clone();
  return om;
}

void FixedRewardObservationModel::step(const Vector &obs, const Vector &action, Vector *next, double *reward, int *terminal) const
{
  ApproximatedObservationModel::step(obs, action, next, reward, terminal);
  
  if (next->empty())
    return;
  
  Vector state, next_state;
  if (!task_->invert(obs, &state))
  {
    WARNING("Task does not support inversion");
    return;
  }
  
  task_->invert(*next, &next_state);
  task_->evaluate(state, action, next_state, reward);
}
