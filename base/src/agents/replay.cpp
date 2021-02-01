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
    agent_->replay();
}

void ReplayAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("memory_size", "Maximum number of transitions in replay memory", memory_size_, CRP::Configuration, 1));
  config->push_back(CRP("replay_steps", "Number of replay steps per control step", replay_steps_, CRP::Online, 1));
  config->push_back(CRP("batch_size", "Number of replay steps per batch", batch_size_, CRP::Configuration, 1));
  config->push_back(CRP("observation_steps", "Number of steps to wait before starting replay", observation_steps_, CRP::Configuration, 1));
  config->push_back(CRP("threads", "Threads used for replay (0 = synchronous replay. >0 requires reentrant predictor)", threads_, CRP::Configuration, 0, INT_MAX));
  
  config->push_back(CRP("observation_policy", "mapping/policy", "Control policy for observation steps", observation_policy_, true));
  config->push_back(CRP("policy", "mapping/policy", "Control policy", policy_));
  config->push_back(CRP("predictor", "predictor", "Value function predictor", predictor_));
}

void ReplayAgent::configure(Configuration &config)
{
  observation_policy_ = (Policy*)config["observation_policy"].ptr();
  policy_ = (Policy*)config["policy"].ptr();
  predictor_ = (Predictor*)config["predictor"].ptr();

  memory_size_ = config["memory_size"];
  replay_steps_ = config["replay_steps"];
  batch_size_ = config["batch_size"];
  observation_steps_ = config["observation_steps"];
  threads_ = config["threads"];
  
  transitions_.reserve(memory_size_);
}

void ReplayAgent::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    stopThreads();
    transitions_.clear();
    total_replay_steps_ = total_control_steps_ = total_transitions_ = 0;
  }
    
  config.get("replay_steps", replay_steps_);
}

void ReplayAgent::start(const Observation &obs, Action *action)
{
  time_= 0.;
  if (total_control_steps_ < observation_steps_ && observation_policy_)
    observation_policy_->act(time_, obs, action);
  else
    policy_->act(time_, obs, action);
  
  prev_obs_ = obs;
  prev_action_ = *action;

  if (threads_ && replay_threads_.empty())
    startThreads();
}

void ReplayAgent::step(double tau, const Observation &obs, double reward, Action *action)
{
  time_ += tau;
  if (total_control_steps_ < observation_steps_ && observation_policy_)
    observation_policy_->act(time_, obs, action);
  else
    policy_->act(time_, obs, action);
  
  // Add new transition to replay memory
  if (transitions_.size() < memory_size_)
    transitions_.push_back(Transition(prev_obs_, prev_action_, tau, reward, obs, *action));
  else
    transitions_[total_transitions_%memory_size_] = Transition(prev_obs_, prev_action_, tau, reward, obs, *action);
    
  total_transitions_++;
  total_control_steps_++;
  
  replay_signal_.signal();
  
  if (!threads_)
    replay();
    
  // Avoid running ahead of replay
  while (total_replay_steps_ < total_control_steps_*replay_steps_)
    done_signal_.read();

  prev_obs_ = obs;
  prev_action_ = *action;  
}

void ReplayAgent::end(double tau, const Observation &obs, double reward)
{
  if (transitions_.size() < memory_size_)
    transitions_.push_back(Transition(prev_obs_, prev_action_, tau, reward, obs));
  else
    transitions_[total_transitions_%memory_size_] = Transition(prev_obs_, prev_action_, tau, reward, obs);

  total_transitions_++;
  total_control_steps_++;
  
  replay_signal_.signal();

  if (!threads_)
    replay();

  // Avoid running ahead of replay
  while (total_replay_steps_ < total_control_steps_*replay_steps_)
    done_signal_.read();
}

void ReplayAgent::report(std::ostream &os)
{
  os << std::setw(15) << total_replay_steps_;
}

void ReplayAgent::replay()
{
  replay_signal_.read();

  if (transitions_.size() < observation_steps_)
  {
    total_replay_steps_ = total_control_steps_*replay_steps_;
    done_signal_.signal();
    return;
  }
  else if (transitions_.size() == observation_steps_)
    INFO("Starting replay");

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
  
  done_signal_.signal();
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
    replay_threads_[ii]->stop();
    replay_signal_.signal();
    replay_threads_[ii]->join();
    safe_delete(&replay_threads_[ii]);
  }
  
  replay_threads_.clear();
}
