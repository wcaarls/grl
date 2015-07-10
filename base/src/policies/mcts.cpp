/** \file mcts.cpp
 * \brief Monte-Carlo Tree Search policy source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-06-12
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

#include <grl/policies/mcts.h>

using namespace grl;

REGISTER_CONFIGURABLE(MCTSPolicy)

void MCTSPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("model", "observation_model", "Observation model used for planning", model_));
  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  
  config->push_back(CRP("epsilon", "Exploration rate", epsilon_));
  config->push_back(CRP("horizon", "Planning horizon", horizon_, CRP::Online));
  config->push_back(CRP("budget", "Computational budget", budget_, CRP::Online));
}

void MCTSPolicy::configure(Configuration &config)
{
  model_ = (ObservationModel*)config["model"].ptr();
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  discretizer_->options(&actions_);

  epsilon_ = config["epsilon"];
  horizon_ = config["horizon"];
  budget_ = config["budget"];
}

void MCTSPolicy::reconfigure(const Configuration &config)
{
  config.get("horizon", horizon_);
  config.get("budget", budget_);
}

MCTSPolicy *MCTSPolicy::clone() const
{
  return NULL;
}

void MCTSPolicy::act(double time, const Vector &in, Vector *out)
{
  // Clear tree at start of episode
  if (time == 0.)
  {
    safe_delete(&root_);
    trunk_ = NULL;
  }

  // Try warm start
  if (trunk_ && trunk_->children())
  {
    double maxdiff = 0;
    MCTSNode *selected = trunk_->select(0);
    Vector predicted = selected->state();
    
    for (size_t ii=0; ii < in.size(); ++ii)
      maxdiff = fmax(maxdiff, fabs(in[ii]-predicted[ii]));
      
    if (maxdiff < 0.05)
    {
      trunk_ = selected;
      selected->orphanize();

      CRAWL("Trunk set to selected state " << trunk_->state());
    }
    else
    {
      safe_delete(&root_);
      trunk_ = NULL;
      DEBUG("Cannot use warm start: predicted state " << predicted << " differs from actual state " << in);
    }
  }

  // Allocate new tree if warm start was not possible
  if (!trunk_)
  {
    allocate();
    root_->init(NULL, 0, in, 0, false);
    root_->allocate(actions_.size());
    trunk_ = root_;
  }
  
  CRAWL("Trunk set to state " << trunk_->state());

  // Search until budget is up
  timer t;
  size_t searches=0;

  while (t.elapsed() < budget_)
  {
    MCTSNode *node = treePolicy();
    double reward = 0;
    
    CRAWL("Tree policy selected node with state " << node->state());
    
    if (!node->terminal())
      reward = defaultPolicy(node->state());
     
    CRAWL("Default policy got reward " << reward);

    do
    {
      node->update(reward);
      reward += node->reward();
    } while ((node = node->parent()));
    
    CRAWL("Backup done");
    searches++;
  }
  
  // Select best action
  if (trunk_->children())
    *out = actions_[trunk_->select(0)->action()];
  else
    *out = actions_[lrand48()%actions_.size()];
  
  CRAWL("Selected action " << *out << " after " << searches << " searches");
}
