/** \file leo_td.cpp
 * \brief Leo temporal difference agent source file.
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

#include <grl/environments/leo/agents/leo_td.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoTDAgent)

void LeoTDAgent::request(ConfigurationRequest *config)
{
  TDAgent::request(config);
  config->push_back(CRP("pub_transition_type", "signal/vector", "Publisher of the transition type", pub_transition_type_, true));
}

void LeoTDAgent::configure(Configuration &config)
{
  TDAgent::configure(config);
  pub_transition_type_ = (VectorSignal*)config["pub_transition_type"].ptr();
}

void LeoTDAgent::reconfigure(const Configuration &config)
{
  TDAgent::reconfigure(config);
}

LeoTDAgent *LeoTDAgent::clone() const
{
  LeoTDAgent *agent = new LeoTDAgent();
  agent->policy_ = policy_->clone();
  agent->predictor_= predictor_->clone();
  return agent;
}

TransitionType LeoTDAgent::start(const Vector &obs, Vector *action)
{
  TransitionType tt = TDAgent::start(obs, action);

  if (pub_transition_type_)
    pub_transition_type_->set(VectorConstructor((double)tt));

  return tt;
}

TransitionType LeoTDAgent::step(double tau, const Vector &obs, double reward, Vector *action)
{
  TransitionType tt = TDAgent::step(tau, obs, reward, action);

  if (pub_transition_type_)
    pub_transition_type_->set(VectorConstructor((double)tt));

  return tt;
}
