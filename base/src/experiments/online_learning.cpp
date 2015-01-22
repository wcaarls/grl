/** \file online_learning.cpp
 * \brief Online learning experiment source file.
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

#include <grl/experiments/online_learning.h>

using namespace grl;

REGISTER_CONFIGURABLE(OnlineLearningExperiment)

void OnlineLearningExperiment::request(ConfigurationRequest *config)
{
}

void OnlineLearningExperiment::configure(Configuration &config)
{
  agent_ = (Agent*)config["agent"].ptr();
  environment_ = (Environment*)config["environment"].ptr();
  
  config.get("runs", runs_, (size_t)1);
  config.get("trials", trials_, (size_t)0);
  config.get("steps", steps_, (size_t)0);
  config.get("rate", rate_, 0.);
}

void OnlineLearningExperiment::reconfigure(const Configuration &config)
{
}

OnlineLearningExperiment *OnlineLearningExperiment::clone() const
{
  OnlineLearningExperiment *ole = new OnlineLearningExperiment();
  
  ole->agent_ = agent_->clone();
  ole->environment_ = environment_->clone();
  ole->runs_ = runs_;
  ole->trials_ = trials_;
  ole->steps_ = steps_;
  
  return ole;
}

void OnlineLearningExperiment::run() const
{
  for (size_t rr=0; rr < runs_; ++rr)
  {
    for (size_t ss=0, tt=0; (!trials_ || tt < trials_) && (!steps_ || ss < steps_); ++tt)
    { 
      Vector obs, action;
      double reward, total_reward=0;
      int terminal;
      
      environment_->start(&obs);
      agent_->start(obs, &action);
      
      do
      {
        if (rate_) usleep(1000000./rate_);
        
//        std::cout << obs << ", " << action << std::endl;
      
        environment_->step(action, &obs, &reward, &terminal);
        total_reward += reward;
        
        if (terminal == 2)
          agent_->end(reward);
        else
          agent_->step(obs, reward, &action);
          
        ++ss;
      } while (!terminal);
      
      std::cout << tt << ", " << ss << ", " << total_reward << std::endl;
    }
  }
}
