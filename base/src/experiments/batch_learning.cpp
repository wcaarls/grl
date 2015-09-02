/** \file batch_learning.cpp
 * \brief Batch learning experiment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-04-29
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

#include <unistd.h>
#include <iostream>
#include <iomanip>

#include <grl/experiments/batch_learning.h>

using namespace grl;

REGISTER_CONFIGURABLE(BatchLearningExperiment)

void BatchLearningExperiment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("runs", "Number of separate learning runs to perform", runs_, CRP::Configuration, 1));
  config->push_back(CRP("batches", "Number of batches per learning run", (int)batches_));
  config->push_back(CRP("batch_size", "Number of transitions per batch", (int)batch_size_));
  config->push_back(CRP("rate", "Test trial control step frequency in Hz", (int)rate_, CRP::Online));
  config->push_back(CRP("output", "Output base filename", output_));
  
  config->push_back(CRP("model", "model", "Model in which the task is set", model_));
  config->push_back(CRP("task", "task", "Task to be solved", task_));
  config->push_back(CRP("predictor", "predictor", "Learner", predictor_));
  config->push_back(CRP("test_agent", "agent", "Agent to use in test trials after each batch", test_agent_));
  
  config->push_back(CRP("observation_min", "vector.observation_min", "Lower limit for observations", observation_min_, CRP::System));
  config->push_back(CRP("observation_max", "vector.observation_max", "Upper limit for observations", observation_max_, CRP::System));
  config->push_back(CRP("action_min", "vector.action_min", "Lower limit for actions", action_min_, CRP::System));
  config->push_back(CRP("action_max", "vector.action_max", "Upper limit for actions", action_max_, CRP::System));
  
  config->push_back(CRP("state", "state", "Current observed state of the environment", CRP::Provided));  
}

void BatchLearningExperiment::configure(Configuration &config)
{
  model_ = (Model*)config["model"].ptr();
  task_ = (Task*)config["task"].ptr();
  predictor_ = (Predictor*)config["predictor"].ptr();
  test_agent_ = (Agent*)config["test_agent"].ptr();
  
  runs_ = config["runs"];
  batches_ = config["batches"];
  batch_size_ = config["batch_size"];
  rate_ = config["rate"];
  output_ = config["output"].str();

  observation_min_ = config["observation_min"]; 
  observation_max_ = config["observation_max"]; 
  action_min_ = config["action_min"]; 
  action_max_ = config["action_max"]; 
  
  state_ = new State();
  
  config.set("state", state_);
}

void BatchLearningExperiment::reconfigure(const Configuration &config)
{
  config.get("rate", rate_);
}

BatchLearningExperiment *BatchLearningExperiment::clone() const
{
  return NULL;
}

void BatchLearningExperiment::run()
{
  std::ofstream ofs;

  for (size_t rr=0; rr < runs_; ++rr)
  {
    if (!output_.empty())
    {
      std::ostringstream oss;
      oss << output_ << "-" << rr << ".txt";
      ofs.open(oss.str().c_str());
    }
    
    for (size_t bb=0; !batches_ || bb < batches_; ++bb)
    { 
      // Create a batch of random experience
      for (size_t ss=0; ss < batch_size_; ++ss)
      {
        Vector obs = RandGen::getVector(observation_min_.size()),
               action = RandGen::getVector(action_min_.size()),
               next_action = RandGen::getVector(action_min_.size());
               
        obs = observation_min_ + obs*(observation_max_-observation_min_);
        action = action_min_ + action*(action_max_-action_min_);
        next_action = action_min_ + next_action*(action_max_-action_min_);
        
        Vector state;
        if (!task_->invert(obs, &state))
        {
          ERROR("Task does not support inversion");
          throw bad_param("experiment/online_learning:task");
        }
        
        Vector next, next_obs;
        int terminal;
        double reward;
        model_->step(state, action, &next);
        task_->observe(next, &next_obs, &terminal);
        task_->evaluate(state, action, next, &reward);
        
        if (terminal == 2)
        {
          next_obs = Vector();
          next_action = Vector();
        }
        
        Transition t(obs, action, reward, next_obs, next_action);
        
        predictor_->update(t);
      }
      
      // Batch learning
      predictor_->finalize();
        
      // Test trial
      Vector state, next, obs, next_obs, action, next_action;
      double reward, total_reward=0;
      int terminal;
      
      task_->start(1, &state);
      state_->set(state);
      task_->observe(state, &obs, &terminal);
      test_agent_->start(obs, &action);

      CRAWL(obs);
  
      do
      {
        if (rate_) usleep(1000000./rate_);
        
        double tau = model_->step(state, action, &next);
        task_->observe(next, &obs, &terminal);
        task_->evaluate(state, action, next, &reward);
        state = next;
        state_->set(state);
        
        CRAWL(action << " - " << reward << " -> " << obs);
        
        total_reward += reward;
        
        if (terminal == 2)
          test_agent_->end(tau, reward);
        else if (!obs.empty())
        {
          test_agent_->step(tau, obs, reward, &action);
        }
      } while (!terminal);

      // Report results
      std::ostringstream oss;
      oss << std::setw(15) << bb << std::setw(15) << bb*batch_size_ << std::setw(15) << total_reward;
      test_agent_->report(oss);
        
      INFO(oss.str());
      if (ofs.is_open())
        ofs << oss.str() << std::endl;
    }
    
    if (ofs.is_open())
      ofs.close();
      
    if (rr < runs_ - 1)
      reset();
  }
}
