/** \file filtering.cpp
 * \brief Filtering agent source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-07-20
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

#include <grl/agents/filtering.h>

using namespace grl;

REGISTER_CONFIGURABLE(FilteringAgent)
REGISTER_CONFIGURABLE(FilteringSubAgent)

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
        throw bad_param("agent/filtering:{observation,action}");
      }
    
      newv[ii] = v[idx];
    }
  }
  
  return newv;
}

// Generate vector inv_idx such that reindex(reindex(v, index), inv_idx) = v,
// with lost dimensions set to 0.
static Vector invert_index(const Vector &index, size_t sz)
{
  Vector inv_idx = ConstantVector(sz, -1);
  
  for (size_t ii=0; ii < sz; ++ii)
    for (size_t jj=0; jj < index.size(); ++jj)
      if (index[jj] == ii)
        inv_idx[ii] = jj;
        
  return inv_idx;
}

// *** FilteringAgent ***

void FilteringAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("observation_idx", "vector", "Index vector for downstream observation (-1=pad)", observation_idx_));
  config->push_back(CRP("action_idx", "vector", "Index vector for upstream action (-1=pad)", action_idx_));
  config->push_back(CRP("action_dims", "int", "Number of downstream action dimensions", action_dims_));
  
  config->push_back(CRP("agent", "agent", "Downstream agent", agent_));
}

void FilteringAgent::configure(Configuration &config)
{
  observation_idx_ = config["observation_idx"].v();
  action_dims_ = config["action_dims"];
  action_idx_ = config["action_idx"].v();
  inv_action_idx_ = invert_index(action_idx_, action_dims_);
  
  agent_ = (Agent*)config["agent"].ptr();
}

void FilteringAgent::reconfigure(const Configuration &config)
{
}

FilteringAgent *FilteringAgent::clone() const
{
  FilteringAgent *agent = new FilteringAgent();
  agent->agent_ = agent_->clone();
  
  return agent;
}

TransitionType FilteringAgent::start(const Vector &obs, Vector *action)
{
  Vector downstream_obs = reindex(obs, observation_idx_), downstream_action;
  
  if (action->size())
    downstream_action = reindex(*action, inv_action_idx_);
  
  TransitionType tt = agent_->start(downstream_obs, &downstream_action);
  
  *action = reindex(downstream_action, action_idx_);

  return tt;
}

TransitionType FilteringAgent::step(double tau, const Vector &obs, double reward, Vector *action)
{
  Vector downstream_obs = reindex(obs, observation_idx_), downstream_action;
  
  if (action->size())
    downstream_action = reindex(*action, inv_action_idx_);
  
  TransitionType tt = agent_->step(tau, downstream_obs, reward, &downstream_action);
  
  *action = reindex(downstream_action, action_idx_);

  return tt;
}

void FilteringAgent::end(double tau, const Vector &obs, double reward)
{
  Vector downstream_obs = reindex(obs, observation_idx_);
  
  agent_->end(tau, downstream_obs, reward);
}

// *** FilteringSubAgent ***

void FilteringSubAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("observation_idx", "vector", "Index vector for downstream observation (-1=pad)", observation_idx_));
  config->push_back(CRP("action_idx", "vector", "Index vector for upstream action (-1=pad)", action_idx_));
  config->push_back(CRP("action_dims", "int", "Number of downstream action dimensions", action_dims_));
  
  config->push_back(CRP("agent", "agent/sub", "Downstream subagent", agent_));
}

void FilteringSubAgent::configure(Configuration &config)
{
  observation_idx_ = config["observation_idx"].v();
  action_dims_ = config["action_dims"];
  action_idx_ = config["action_idx"].v();
  inv_action_idx_ = invert_index(action_idx_, action_dims_);

  agent_ = (SubAgent*)config["agent"].ptr();
}

void FilteringSubAgent::reconfigure(const Configuration &config)
{
}

FilteringSubAgent *FilteringSubAgent::clone() const
{
  FilteringSubAgent *agent = new FilteringSubAgent();
  agent->agent_ = agent_->clone();
  
  return agent;
}

TransitionType FilteringSubAgent::start(const Vector &obs, Vector *action, double *conf)
{
  Vector downstream_obs = reindex(obs, observation_idx_), downstream_action;
  
  if (action->size())
    downstream_action = reindex(*action, inv_action_idx_);
  
  TransitionType tt = agent_->start(downstream_obs, &downstream_action, conf);
  
  *action = reindex(downstream_action, action_idx_);

  return tt;
}

TransitionType FilteringSubAgent::step(double tau, const Vector &obs, double reward, Vector *action, double *conf)
{
  Vector downstream_obs = reindex(obs, observation_idx_), downstream_action;
  
  if (action->size())
    downstream_action = reindex(*action, inv_action_idx_);
  
  TransitionType tt = agent_->step(tau, downstream_obs, reward, &downstream_action, conf);
  
  *action = reindex(downstream_action, action_idx_);

  return tt;
}

void FilteringSubAgent::end(double tau, const Vector &obs, double reward)
{
  Vector downstream_obs = reindex(obs, observation_idx_);
  
  agent_->end(tau, downstream_obs, reward);
}

double FilteringSubAgent::confidence(const Vector &obs) const
{
  Vector downstream_obs = reindex(obs, observation_idx_);

  return agent_->confidence(downstream_obs);
}
