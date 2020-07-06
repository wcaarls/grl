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

#include <chrono>
#include <iostream>
#include <iomanip>

#include <grl/experiments/online_learning.h>

using namespace grl;

REGISTER_CONFIGURABLE(OnlineLearningExperiment)

void OnlineLearningExperiment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("runs", "Number of separate learning runs to perform", runs_, CRP::Configuration, 1));
  config->push_back(CRP("run_offset", "Run offset to start at", (int)run_offset_));
  config->push_back(CRP("trials", "Number of episodes per learning run", (int)trials_));
  config->push_back(CRP("steps", "Number of steps per learning run", (int)steps_));
  config->push_back(CRP("rate", "Control step frequency in Hz", (int)rate_, CRP::Online));
  config->push_back(CRP("test_interval", "Number of episodes in between test trials", test_interval_, CRP::Configuration, -1));
  config->push_back(CRP("output", "Output base filename", output_));
  
  config->push_back(CRP("environment", "environment", "Environment in which the agent acts", environment_));
  config->push_back(CRP("agent", "agent", "Agent", agent_));
  config->push_back(CRP("test_agent", "agent", "Agent to use in test trials", agent_, true));
  
  config->push_back(CRP("state", "signal/vector.observation", "Current observed state of the environment", CRP::Provided));
  config->push_back(CRP("action", "signal/vector.action", "Last action applied to the environment", CRP::Provided));
  config->push_back(CRP("reward", "signal/vector.reward", "Last reward received from the environment", CRP::Provided));
  config->push_back(CRP("test_action", "signal/vector.action", "Last action applied to the test environment", CRP::Provided));
  config->push_back(CRP("curve", "signal/vector", "Learning curve", CRP::Provided));

  config->push_back(CRP("load_file", "Load policy filename", load_file_));
  config->push_back(CRP("save_every", "Save policy to 'output' at the end of event", save_every_, CRP::Configuration, {"never", "run", "test", "trial"}));
}

void OnlineLearningExperiment::configure(Configuration &config)
{
  agent_ = (Agent*)config["agent"].ptr();
  test_agent_ = (Agent*)config["test_agent"].ptr();
  environment_ = (Environment*)config["environment"].ptr();
  
  runs_ = config["runs"];
  run_offset_ = config["run_offset"];
  trials_ = config["trials"];
  steps_ = config["steps"];
  rate_ = config["rate"];
  test_interval_ = config["test_interval"];
  output_ = config["output"].str();
  load_file_ = config["load_file"].str();
  save_every_ = config["save_every"].str();
  
  state_ = new VectorSignal();
  action_ = new VectorSignal();
  reward_ = new VectorSignal();
  test_action_ = new VectorSignal();
  curve_ = new VectorSignal();
  
  config.set("state", state_);
  config.set("action", action_);
  config.set("reward", reward_);
  config.set("test_action", test_action_);
  config.set("curve", curve_);

  if (test_interval_ >= 0 && !test_agent_)
    throw bad_param("experiment/online_learning:test_agent");
}

void OnlineLearningExperiment::reconfigure(const Configuration &config)
{
  config.get("rate", rate_);
  config.get("identity", identity_);
}

LargeVector OnlineLearningExperiment::run()
{
  std::ofstream ofs;
  std::vector<double> curve;
  double avg1=0, avg2=0, avgavg1=0, avgavg2=0;
  
  // Store configuration with output
  if (!output_.empty())
  {
    ofs.open(output_ + identity_ + ".yaml");
    ofs << configurator()->root()->yaml(0, true);
    ofs.close();
  }

  for (size_t rr=run_offset_; rr < runs_+run_offset_; ++rr)
  {
    curve.clear();

    if (!output_.empty())
    {
      std::ostringstream oss;
      oss << output_ << "-" << rr << identity_ << ".txt";
      ofs.open(oss.str().c_str());
    }

    // Load policy every run
    if (!load_file_.empty())
    {
      std::string load_file = load_file_ + "-";
      str_replace(load_file, "$run", std::to_string((int)rr)); // increment run if needed
      std::cout << "Loading policy: " << load_file << std::endl;
      Configuration loadconfig;
      loadconfig.set("action", "load");
      loadconfig.set("file", load_file );
      agent_->walk(loadconfig);
    }
    
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    for (size_t ss=0, tt=0; (!trials_ || tt < trials_) && (!steps_ || ss < steps_); ++tt)
    { 
      Observation obs;
      Action action;
      double reward, total_reward=0, total_time=0;
      int terminal;
      int test = (test_interval_ >= 0 && tt%(test_interval_+1) == test_interval_) * (rr+1);
      timer step_timer;

      Agent *agent = agent_;      
      if (test) agent = test_agent_;
      
      environment_->start(test, &obs);
      state_->set(obs.v);
      reward_->set(VectorConstructor(0.));

      CRAWL(obs);
      
      agent->start(obs, &action);
      action_->set(action.v);
      if (test)
        test_action_->set(action.v);

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
        state_->set(obs.v);
        reward_->set(VectorConstructor(reward));

        CRAWL(action << " - " << reward << " -> " << obs);
        
        total_reward += reward;
        total_time += tau;

        if (obs.size())
        {
          if (terminal == 2)
            agent->end(tau, obs, reward);
          else
            agent->step(tau, obs, reward, &action);

          action_->set(action.v);
          if (test)
            test_action_->set(action.v);
          
          if (!test) ss++;
        }
      } while (!terminal);
      
      std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
      double duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()/1000000.;

      if (test_interval_ >= 0)
      {
        if (test)
        {
          std::ostringstream oss;
          oss << std::setw(15) << tt+1-(tt+1)/(test_interval_+1) << std::setw(15) << ss << std::setw(15) << std::setprecision(3) << std::fixed << total_reward << std::setw(15) << std::setprecision(3) << total_time << std::setw(15) << std::setprecision(3) << total_reward/total_time << std::setw(15) << std::setprecision(3) << duration;
          agent_->report(oss);
          environment_->report(oss);
        
          INFO(oss.str());
          if (ofs.is_open())
            ofs << oss.str() << std::endl;
        }
      }
      else
      {
        std::ostringstream oss;
        oss << std::setw(15) << tt << std::setw(15) << ss << std::setw(15) << std::setprecision(3) << std::fixed << total_reward << std::setw(15) << std::setprecision(3) << total_time << std::setw(15) << std::setprecision(3) << duration;
        agent_->report(oss);
        environment_->report(oss);

        INFO(oss.str());
        if (ofs.is_open())
          ofs << oss.str() << std::endl;
      }
      
      if (test_interval_ < 0 || test)
      {
        // Send out reward signal for displaying learning curve
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

      // Save policy every trial or every test trial
      if (((save_every_ == "trial") || (test && save_every_ == "test")) && !output_.empty() )
      {
        std::ostringstream oss;
        oss << output_ << "-run" << rr << "-trial" << tt << "-";
        Configuration saveconfig;
        saveconfig.set("action", "save");
        saveconfig.set("file", oss.str().c_str());
        agent_->walk(saveconfig);
      }
    }
    
    // Save policy every run
    if (save_every_ == "run" && !output_.empty())
    {
      std::ostringstream oss;
      oss << output_ << "-run" << rr << "-";
      Configuration saveconfig;
      saveconfig.set("action", "save");
      saveconfig.set("file", oss.str().c_str());
      agent_->walk(saveconfig);
    }

    if (ofs.is_open())
      ofs.close();
      
    if (rr < runs_ + run_offset_ - 1)
      reset();
  }

  LargeVector result;
  toVector(curve, result);

  return result;
}
