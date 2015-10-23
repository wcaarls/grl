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

#include <unistd.h>
#include <iostream>
#include <iomanip>

#include <grl/experiments/online_learning.h>

using namespace grl;

REGISTER_CONFIGURABLE(OnlineLearningExperiment)

void OnlineLearningExperiment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("runs", "Number of separate learning runs to perform", runs_, CRP::Configuration, 1));
  config->push_back(CRP("trials", "Number of episodes per learning run", (int)trials_));
  config->push_back(CRP("steps", "Number of steps per learning run", (int)steps_));
  config->push_back(CRP("rate", "Control step frequency in Hz", (int)rate_, CRP::Online));
  config->push_back(CRP("test_interval", "Number of episodes in between test trials", test_interval_, CRP::Configuration, -1));
  config->push_back(CRP("output", "Output base filename", output_));
  
  config->push_back(CRP("environment", "environment", "Environment in which the agent acts", environment_));
  config->push_back(CRP("agent", "agent", "Agent", agent_));
  config->push_back(CRP("test_agent", "agent", "Agent to use in test trials", agent_, true));
  
  config->push_back(CRP("state", "state", "Current observed state of the environment", CRP::Provided));
  config->push_back(CRP("curve", "state", "Learning curve", CRP::Provided));

  config->push_back(CRP("load_file", "Load policy filename", load_file_));
  config->push_back(CRP("save_every", "Save policy to 'output' at the end of event", save_every_, CRP::Configuration, {"never", "run", "test", "trail"}));
}

void OnlineLearningExperiment::configure(Configuration &config)
{
  agent_ = (Agent*)config["agent"].ptr();
  test_agent_ = (Agent*)config["test_agent"].ptr();
  environment_ = (Environment*)config["environment"].ptr();
  
  runs_ = config["runs"];
  trials_ = config["trials"];
  steps_ = config["steps"];
  rate_ = config["rate"];
  test_interval_ = config["test_interval"];
  output_ = config["output"].str();
  load_file_ = config["load_file"].str();
  save_every_ = config["save_every"].str();
  
  state_ = new State();
  curve_ = new State();
  
  config.set("state", state_);
  config.set("curve", curve_);

  if (test_interval_ >= 0 && !test_agent_)
    throw bad_param("experiment/online_learning:test_agent");
}

void OnlineLearningExperiment::reconfigure(const Configuration &config)
{
  config.get("rate", rate_);
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

void OnlineLearningExperiment::run()
{
  std::ofstream ofs;

  for (size_t rr=0; rr < runs_; ++rr)
  {
    // Load policy/store
    if (!load_file_.empty())
    {
      Configuration loadconfig;
      loadconfig.set("action", "load");
      loadconfig.set("file", load_file_ + "-" + std::to_string((int)rr) + "-");
       agent_->walk(loadconfig);
    }

    if (!output_.empty())
    {
      std::ostringstream oss;
      oss << output_ << "-" << rr << ".txt";
      ofs.open(oss.str().c_str());
    }
    
    for (size_t ss=0, tt=0; (!trials_ || tt < trials_) && (!steps_ || ss < steps_); ++tt)
    { 
      Vector obs, action;
      double reward, total_reward=0;
      int terminal;
      bool test = (test_interval_ >= 0 && tt%(test_interval_+1) == test_interval_);
      timer step_timer;

      Agent *agent = agent_;      
      if (test) agent = test_agent_;
      
      environment_->start(test, &obs);
      agent->start(obs, &action);
      state_->set(obs);

      CRAWL(obs);
  
      do
      {
        if (rate_)
        {
          double sleep_time = 1./rate_-step_timer.elapsed();
          if (sleep_time > 0)
            usleep(1000000.*sleep_time);
          step_timer.restart();
        }
        
        double tau = environment_->step(action, &obs, &reward, &terminal);
        
        CRAWL(action << " - " << reward << " -> " << obs);
        
        total_reward += reward;
        
        if (terminal == 2)
          agent->end(tau, reward);
        else if (obs.size())
        {
          agent->step(tau, obs, reward, &action);
          state_->set(obs);
        }
          
        if (!test && obs.size()) ss++;
      } while (!terminal);

      if (test_interval_ >= 0)
      {
        if (test)
        {
          std::ostringstream oss;
          oss << std::setw(15) << tt+1-(tt+1)/(test_interval_+1) << std::setw(15) << ss << std::setw(15) << total_reward;
          agent_->report(oss);
          curve_->set(VectorConstructor(total_reward));
        
          INFO(oss.str());
          if (ofs.is_open())
            ofs << oss.str() << std::endl;
        }
      }
      else
      {
        std::ostringstream oss;
        oss << std::setw(15) << tt << std::setw(15) << ss << std::setw(15) << total_reward;
        agent_->report(oss);
        curve_->set(VectorConstructor(total_reward));
        
        INFO(oss.str());
        if (ofs.is_open())
          ofs << oss.str() << std::endl;
      }

      // Save policy/store every trial or every test trial
      if (((save_every_ == "trial") || (test && save_every_ == "test")) && !output_.empty() )
      {
        std::ostringstream oss;
        oss << output_ << "-" << rr << "-" << ss << "-";
        Configuration saveconfig;
        saveconfig.set("action", "save");
        saveconfig.set("file", oss.str().c_str());
        agent_->walk(saveconfig);
      }
    }
    
    // Save policy/store every run
    if (save_every_ == "run" && !output_.empty())
    {
      std::ostringstream oss;
      oss << output_ << "-" << rr << "-";
      Configuration saveconfig;
      saveconfig.set("action", "save");
      saveconfig.set("file", oss.str().c_str());
      agent_->walk(saveconfig);
    }

    if (ofs.is_open())
      ofs.close();
      
    if (rr < runs_ - 1)
      reset();
  }
}
