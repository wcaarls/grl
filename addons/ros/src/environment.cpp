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

  config->push_back(CRP("observation_dims", "int.observation_dims", "Number of observation dimensions", CRP::Provided));
  config->push_back(CRP("observation_min", "vector.observation_min", "Lower limit on observations", CRP::Provided));
  config->push_back(CRP("observation_max", "vector.observation_max", "Upper limit on observations", CRP::Provided));
  config->push_back(CRP("action_dims", "int.action_dims", "Number of action dimensions", CRP::Provided));
  config->push_back(CRP("action_min", "vector.action_min", "Lower limit on actions", CRP::Provided));
  config->push_back(CRP("action_max", "vector.action_max", "Upper limit on actions", CRP::Provided));
  config->push_back(CRP("reward_min", "double.reward_min", "Lower limit on immediate reward", CRP::Provided));
  config->push_back(CRP("reward_max", "double.reward_max", "Upper limit on immediate reward", CRP::Provided));
}

void ROSEnvironment::configure(Configuration &config)
{
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
  
  spinner_ = new ros::AsyncSpinner(1);
  spinner_->start();
  
  // Wait for description
  NOTICE("Waiting for description from ROS environment");
  mprl_msgs::EnvDescription msg = desc_reader_++;
  
  Vector v;

  toVector(msg.observation_min, v);
  state_dims_ = v.size();
  config.set("observation_dims", state_dims_);
  config.set("observation_min", v);
  toVector(msg.observation_max, v);
  config.set("observation_max", v);
  toVector(msg.action_min, v);
  config.set("action_dims", v.size());
  config.set("action_min", v);
  toVector(msg.action_max, v);
  config.set("action_max", v);

  config.set("reward_min", msg.reward_min);
  config.set("reward_max", msg.reward_max);

  config.set("stochastic", msg.stochastic);
  config.set("episodic", msg.episodic);
  config.set("title", msg.title);
}

void ROSEnvironment::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    // TODO: send reset message to ROS agent
  }
}

ROSEnvironment *ROSEnvironment::clone() const
{
  return NULL;
}

void ROSEnvironment::start(int test, Vector *obs)
{
  if (running_)
    throw Exception("Cannot start environment because ROS environment is still running");

  mprl_msgs::StateReward msg = state_reader_++;
  toVector(msg.state, *obs);
  
  running_ = true;
}

void ROSEnvironment::step(const Vector &action, Vector *obs, double *reward, int *terminal)
{
  if (!running_)
    throw Exception("Cannot step environment because ROS environment is not running");
  
  mprl_msgs::Action actionmsg;
  fromVector(action, actionmsg.action);
  action_pub_.publish(actionmsg);

  mprl_msgs::StateReward msg = state_reader_++;
  toVector(msg.state, *obs);
  *reward = msg.reward;
  *terminal = msg.terminal + msg.absorbing;
  
  if (*terminal)
    running_ = false;
}

void ROSEnvironment::callbackDesc(const mprl_msgs::EnvDescription::ConstPtr &descmsg)
{
  desc_writer_ += *descmsg;
}

void ROSEnvironment::callbackState(const mprl_msgs::StateReward::ConstPtr &statemsg)
{
  Vector state;
  toVector(statemsg->state, state);
  
  if (state.size() && state_dims_ && state.size() != state_dims_)
    WARNING("Observation received through ROS (size " << state.size() << ") is not compatible with description (size " << state_dims_ << ")");
      
  state_writer_ += *statemsg; 
}
