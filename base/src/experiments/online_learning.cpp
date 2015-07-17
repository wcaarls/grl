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
  config->push_back(CRP("test_interval", "Number of episodes in between test trials", (int)test_interval_));
  config->push_back(CRP("output", "Output base filename", output_));
  
  config->push_back(CRP("environment", "environment", "Environment in which the agent acts", environment_));
  config->push_back(CRP("agent", "agent", "Agent", agent_));
  config->push_back(CRP("test_agent", "agent", "Agent to use in test trials", agent_, true));
  
  config->push_back(CRP("state", "state", "Current observed state of the environment", CRP::Provided));  
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
  
  state_ = new State();
  
  config.set("state", state_);
  
  if (test_interval_ && !test_agent_)
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
    if (!output_.empty())
    {
      std::ostringstream oss;
      oss << output_ << "-" << rr;
      ofs.open((oss.str()+".txt").c_str());
      csv_.start((oss.str()+".csv").c_str(), 7, 1, 1);
    }
    
    for (size_t ss=0, tt=0; (!trials_ || tt < trials_) && (!steps_ || ss < steps_); ++tt)
    { 
      Vector obs, action;
      double reward, total_reward=0;
      int terminal;
      bool test = (test_interval_ && tt%(test_interval_+1) == test_interval_);

      Agent *agent = agent_;      
      if (test) agent = test_agent_;
      
      environment_->start(test, &obs);
      //obs.pop_back(); // TODO: Hack to record csv file. Remove asap.
      agent->start(obs, &action);
      state_->set(obs);

      CRAWL(obs);
  
      do
      {
        if (rate_) usleep(1000000./rate_);
        
        environment_->step(action, &obs, &reward, &terminal);

        CRAWL(action << " - " << reward << " -> " << obs);

        //obs.push_back(-obs.back()*sin(0.004)); // TODO: Hack to record csv file. Remove asap.
        csv_.log(0.2, obs, action, reward);
        //obs.pop_back(); // TODO: Hack to record csv file. Remove asap.
        //obs.pop_back();

        total_reward += reward;
        
        if (terminal == 2)
          agent->end(reward);
        else if (!obs.empty())
        {
          agent->step(obs, reward, &action);
          state_->set(obs);
        }
          
        if (!test && !obs.empty()) ss++;
      } while (!terminal);

      if (test_interval_)
      {
        if (test)
        {
          std::ostringstream oss;
          oss << std::setw(15) << tt+1-(tt+1)/(test_interval_+1) << std::setw(15) << ss << std::setw(15) << total_reward;
          agent_->report(oss);
        
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
        
        INFO(oss.str());
        if (ofs.is_open())
          ofs << oss.str() << std::endl;
      }
    }
    
    if (ofs.is_open())
      ofs.close();

    csv_.stop();
      
    if (rr < runs_ - 1)
      reset();
  }
}
