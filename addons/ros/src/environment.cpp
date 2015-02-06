/** \file environment.cpp
 * \brief ROS environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-05
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

#include <grl/environments/ros.h>

using namespace grl;

REGISTER_CONFIGURABLE(ROSEnvironment)

void ROSEnvironment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("node", "ROS node name", node_));
  config->push_back(CRP("args", "ROS command-line arguments", args_));
}

void ROSEnvironment::configure(Configuration &config)
{
  Guard guard(mutex_);

  node_ = config["node"].str();
  args_ = config["args"].str();
  
  // Parse arguments
  int argc=1;
  char *argv[] = {"deployer"};
  
  ros::init(argc, argv, node_.c_str());
  
  nh_agent_ = new ros::NodeHandle("rl_agent");
  nh_env_ = new ros::NodeHandle("rl_env");
  
  action_pub_ = nh_agent_->advertise<mprl_msgs::Action> ("rl_action", 10, true);
  state_sub_ = nh_env_->subscribe("rl_state_reward", 10, &ROSEnvironment::callbackState, this);
  desc_sub_ = nh_env_->subscribe("rl_env_description", 10, &ROSEnvironment::callbackDesc, this);
  
  spinner_ = new ros::AsyncSpinner(0);
  spinner_->start();
  
  // Wait for description
  NOTICE("Waiting for description from ROS environment");
  new_description_.wait(mutex_);
  
  Vector v;

  toVector(description_.observation_min, v);
  
  std::cout << description_.observation_min << std::endl;
  
  state_dims_ = v.size();
  config.set("observation_min", v);
  toVector(description_.observation_max, v);
  config.set("observation_max", v);
  toVector(description_.action_min, v);
  config.set("action_min", v);
  toVector(description_.action_max, v);
  config.set("action_max", v);

  config.set("reward_min", description_.reward_min);
  config.set("reward_max", description_.reward_max);

  config.set("stochastic", description_.stochastic);
  config.set("episodic", description_.episodic);
  config.set("title", description_.title);
}

void ROSEnvironment::reconfigure(const Configuration &config)
{
}

ROSEnvironment *ROSEnvironment::clone() const
{
  return NULL;
}

void ROSEnvironment::start(Vector *obs)
{
  Guard guard(mutex_);

  if (running_)
    throw Exception("Cannot start environment because ROS environment is still running");
    
  if (state_.empty())
    new_state_.wait(mutex_);
  
  *obs = state_;
  
  running_ = true;
}

void ROSEnvironment::step(const Vector &action, Vector *obs, double *reward, int *terminal)
{
  Guard guard(mutex_);

  if (!running_)
    throw Exception("Cannot step environment because ROS environment is not running");
  
  mprl_msgs::Action actionmsg;
  fromVector(action, actionmsg.action);
  action_pub_.publish(actionmsg);

  new_state_.wait(mutex_);

  *obs = state_;
  *reward = reward_; 
  *terminal = terminal_;
  
  if (*terminal)
  {
    running_ = false;
    state_.clear();
  }
}

void ROSEnvironment::callbackDesc(const mprl_msgs::EnvDescription::ConstPtr &descmsg)
{
  Guard guard(mutex_);
  
  description_ = *descmsg;
  
  new_description_.signal();
}

void ROSEnvironment::callbackState(const mprl_msgs::StateReward::ConstPtr &statemsg)
{
  Guard guard(mutex_);
  
  toVector(statemsg->state, state_);
  
  if (state_.size() != state_dims_)
    WARNING("Observation received through ROS (size " << state_.size() << ") is not compatible with description (size " << state_dims_ << ")");
      
  reward_ = statemsg->reward;
  terminal_ = statemsg->terminal + statemsg->absorbing;
  
  new_state_.signal();
}
