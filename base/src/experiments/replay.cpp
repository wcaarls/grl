/** \file replay.cpp
 * \brief Experiment replay source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2021-02-02
 *
 * \copyright \verbatim
 * Copyright (c) 2021, Wouter Caarls
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

#include <chrono>
#include <iostream>
#include <iomanip>

#include <grl/experiments/replay.h>

using namespace grl;

REGISTER_CONFIGURABLE(ReplayExperiment)

void ReplayExperiment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("importer", "importer.static", "Importer for transitions (time, state, observation, action, reward, terminal)", importer_));
  config->push_back(CRP("skip", "Number of episodes to skip", (int)skip_));
  config->push_back(CRP("rate", "Control step frequency in Hz", (int)rate_, CRP::Online));
  
  config->push_back(CRP("state", "signal/vector.observation", "Current observed state of the environment", CRP::Provided));
  config->push_back(CRP("action", "signal/vector.action", "Last action applied to the environment", CRP::Provided));
  config->push_back(CRP("reward", "signal/vector.reward", "Last reward received from the environment", CRP::Provided));
  config->push_back(CRP("curve", "signal/vector", "Learning curve", CRP::Provided));
}

void ReplayExperiment::configure(Configuration &config)
{
  importer_ = (Importer*)config["importer"].ptr();
  skip_ = config["skip"];
  rate_ = config["rate"];
  
  state_ = new VectorSignal();
  action_ = new VectorSignal();
  reward_ = new VectorSignal();
  curve_ = new VectorSignal();
  
  config.set("state", state_);
  config.set("action", action_);
  config.set("reward", reward_);
  config.set("curve", curve_);
}

void ReplayExperiment::reconfigure(const Configuration &config)
{
  config.get("rate", rate_);
}

LargeVector ReplayExperiment::run()
{
  Vector time, state, observation, action, reward, terminal;
  std::vector<Vector*> vec {&time, &state, &observation, &action, &reward, &terminal};

  importer_->init({"time", "state", "observation", "action", "reward", "terminal"});
  importer_->open();
  
  std::vector<double> curve;
  Vector prev_state, prev_action;
  double total_reward=0, start_time=0;
  double avg1=0, avg2=0, avgavg1=0, avgavg2=0;
  int tt=0, ss=0;
  timer step_timer;
  
  while (importer_->read(vec))
  {
    if (tt >= skip_)
    {
      CRAWL(time[0] << ", [" << state << "], [" << observation << "], [" << action << "], " << reward[0] << ", " << terminal[0]);
  
      state_->set(state);
      action_->set(action);
      reward_->set(reward);
    
      total_reward += reward[0];

      if (rate_)
      {
        double sleep_time = 1./rate_-step_timer.elapsed();
        if (sleep_time > 0)
          usleep(1000000.*sleep_time);
        step_timer.restart();
      }
    
      if (terminal[0])
      {
        double total_time = time[0]-start_time;
        
        std::cout << std::setw(15) << tt << std::setw(15) << ss << std::setw(15) << std::setprecision(3) << std::fixed << total_reward << std::setw(15) << std::setprecision(3) << total_time << std::setw(15) << std::setprecision(3) << total_reward / total_time << std::endl;
    
        if (curve.empty())
        {
          avg1 = avg2 = total_reward;
          avgavg1 = avgavg2 = total_reward;
        }
        avg1 = 0.1*total_reward + 0.9*avg1;
        avg2 = 0.01*total_reward + 0.99*avg2;
        avgavg1 = 0.1*total_reward/total_time + 0.9*avgavg1;
        avgavg2 = 0.01*total_reward/total_time + 0.99*avgavg2;
        
        curve.push_back(total_reward);
        curve_->set(VectorConstructor(total_reward, avg1, avg2, total_reward/total_time, avgavg1, avgavg2));
      }
    }
    
    ss++;
    if (terminal[0])
    {
      total_reward = 0;
      start_time = time[0];
      tt++;
    }
  }

  LargeVector result;
  toVector(curve, result);

  return result;
}
