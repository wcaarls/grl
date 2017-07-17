/** \file replay.cpp
 * \brief Experience replay agent source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-07-14
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

#include <unistd.h>
#include <iomanip>

#include <grl/agents/replay.h>

using namespace grl;

REGISTER_CONFIGURABLE(ReplayAgent)

void ReplayAgentThread::run()
{
  while (ok())
  {
    agent_->replay();
    usleep(0);
  }
}

void ReplayAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("replay_steps", "Number of replay steps per control step", replay_steps_, CRP::Online, 1));
  config->push_back(CRP("batch_size", "Number of replay steps per batch", batch_size_, CRP::Online, 1));
  config->push_back(CRP("threads", "Threads used for replay (0 = synchronous replay. >0 requires reentrant predictor)", threads_, CRP::Configuration, 0, INT_MAX));
  
  config->push_back(CRP("policy", "mapping/policy", "Control policy", policy_));
  config->push_back(CRP("predictor", "predictor", "Value function predictor", predictor_));
}

void ReplayAgent::configure(Configuration &config)
{
  policy_ = (Policy*)config["policy"].ptr();
  predictor_ = (Predictor*)config["predictor"].ptr();

  replay_steps_ = config["replay_steps"];
  batch_size_ = config["batch_size"];
  threads_ = config["threads"];
}

void ReplayAgent::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    stopThreads();
    total_replay_steps_ = total_control_steps_ = 0;
  }
    
  config.get("replay_steps", replay_steps_);
}

void ReplayAgent::start(const Observation &obs, Action *action)
{
  time_= 0.;
  policy_->act(time_, obs, action);
  
  prev_obs_ = obs;
  prev_action_ = *action;

  if (threads_ && replay_threads_.empty())
    startThreads();
}

void ReplayAgent::step(double tau, const Observation &obs, double reward, Action *action)
{
  time_ += tau;
  policy_->act(time_, obs, action);
  
  // Add new transition to replay memory
  transitions_.push_back(Transition(prev_obs_, prev_action_, reward, obs, *action));
  
  if (!threads_)
    replay();

  prev_obs_ = obs;
  prev_action_ = *action;  

  total_control_steps_++;
}

void ReplayAgent::end(double tau, const Observation &obs, double reward)
{
  transitions_.push_back(Transition(prev_obs_, prev_action_, reward, obs));
  
  if (!threads_)
    replay();

  total_control_steps_++;
}

void ReplayAgent::report(std::ostream &os)
{
  os << std::setw(15) << total_replay_steps_;
}

void ReplayAgent::replay()
{
  while (total_replay_steps_ < total_control_steps_*replay_steps_)
  {
    if (batch_size_ > 1)
    {
      // Sample minibatch
      std::vector<const Transition*> batch(batch_size_);
      for (size_t ii=0; ii < batch_size_; ++ii)
        batch[ii] = &transitions_[RandGen::getInteger(transitions_.size())];
        
      // Update
      predictor_->update(batch);
    }
    else
      predictor_->update(transitions_[RandGen::getInteger(transitions_.size())]);
      
    predictor_->finalize();
    total_replay_steps_ += batch_size_;
  }
}

void ReplayAgent::startThreads()
{
  for (size_t ii=0; ii < threads_; ++ii)
  {
    replay_threads_.push_back(new ReplayAgentThread(this));
    replay_threads_.back()->start();
  }
}

void ReplayAgent::stopThreads()
{
  for (size_t ii=0; ii < replay_threads_.size(); ++ii)
  {
    replay_threads_[ii]->stopAndJoin();
    safe_delete(&replay_threads_[ii]);
  }
  
  replay_threads_.clear();
}
