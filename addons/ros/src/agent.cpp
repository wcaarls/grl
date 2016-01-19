/** \file agent.cpp
 * \brief ROS agent source file.
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

#include <grl/agents/ros.h>

using namespace grl;

REGISTER_CONFIGURABLE(ROSAgent)

void ROSAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("node", "ROS node name", node_));
  config->push_back(CRP("args", "ROS command-line arguments", args_));
  config->push_back(CRP("title", "Name of environment", title_));
  
  config->push_back(CRP("observation_min", "vector.observation_min", "Lower limit on observations", observation_min_, CRP::System));
  config->push_back(CRP("observation_max", "vector.observation_max", "Upper limit on observations", observation_max_, CRP::System));
  
  config->push_back(CRP("action_min", "vector.action_min", "Lower limit on actions", action_min_, CRP::System));
  config->push_back(CRP("action_max", "vector.action_max", "Upper limit on actions", action_max_, CRP::System));
  
  config->push_back(CRP("reward_min", "double.reward_min", "Lower limit on rewards", reward_min_, CRP::System, -DBL_MAX, DBL_MAX));
  config->push_back(CRP("reward_max", "double.reward_max", "Upper limit on rewards", reward_max_, CRP::System, -DBL_MAX, DBL_MAX));

  config->push_back(CRP("stochastic", "Whether the environment is stochastic", stochastic_, CRP::System, 0, 1));
  config->push_back(CRP("episodic", "Whether the environment is episodic", episodic_, CRP::System, 0, 1));
}

void ROSAgent::configure(Configuration &config)
{
  node_ = config["node"].str();
  args_ = config["args"].str();
  
  // Parse arguments
  int argc=1;
  char *argv[] = {"deployer"};
  
  ros::init(argc, argv, node_.c_str());
  
  nh_agent_ = new ros::NodeHandle("rl_agent");
  nh_env_ = new ros::NodeHandle("rl_env");
  
  action_sub_ = nh_agent_->subscribe("rl_action", 10, &ROSAgent::callbackAction, this);
  state_pub_ = nh_env_->advertise<mprl_msgs::StateReward>("rl_state_reward", 10, true);
  desc_pub_ = nh_env_->advertise<mprl_msgs::EnvDescription>("rl_env_description", 10, true);
  
  spinner_ = new ros::AsyncSpinner(1);
  spinner_->start();

  mprl_msgs::EnvDescription descmsg;

  observation_min_ = config["observation_min"];
  fromVector(observation_min_, descmsg.observation_min);
  observation_max_ = config["observation_max"];
  fromVector(observation_max_, descmsg.observation_max);

  action_min_ = config["action_min"];
  action_dims_ = action_min_.size();
  fromVector(action_min_, descmsg.action_min);
  action_max_ = config["action_max"];
  fromVector(action_max_, descmsg.action_max);

  descmsg.reward_min = reward_min_ = config["reward_min"];
  descmsg.reward_max = reward_max_ = config["reward_max"];

  descmsg.stochastic = stochastic_ = config["stochastic"];
  descmsg.episodic = episodic_ = config["episodic"];
  descmsg.title = title_ = config["title"].str();

  NOTICE("Publishing environment description to ROS");

  desc_pub_.publish(descmsg);
  
  // Give agent some time to reinitialize
  usleep(100000);  
}

void ROSAgent::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    // TODO: send reset message to ROS agent
  }
}

ROSAgent *ROSAgent::clone() const
{
  return NULL;
}

void ROSAgent::start(const Vector &obs, Vector *action)
{
  if (running_)
  {
    mprl_msgs::StateReward statemsg;
    statemsg.reward = 0;
    statemsg.terminal = true;
    statemsg.absorbing = false;
    state_pub_.publish(statemsg);
  }	
  
  step(0, obs, 0, action);
  running_ = true;
}

void ROSAgent::step(double tau, const Vector &obs, double reward, Vector *action)
{
  mprl_msgs::StateReward statemsg;
  statemsg.state.resize(obs.size());
  for (size_t ii=0; ii < obs.size(); ++ii)
    statemsg.state[ii] = obs[ii];
  statemsg.reward = reward;
  statemsg.terminal = false;
  statemsg.absorbing = false;

  state_pub_.publish(statemsg);
  
  *action = action_reader_++;
}

void ROSAgent::end(double tau, const Vector &obs, double reward)
{
  mprl_msgs::StateReward statemsg;
  statemsg.state.resize(obs.size());
  for (size_t ii=0; ii < obs.size(); ++ii)
    statemsg.state[ii] = obs[ii];
  statemsg.reward = reward;
  statemsg.terminal = true;
  statemsg.absorbing = true;

  state_pub_.publish(statemsg);
  running_ = false;
}

void ROSAgent::callbackAction(const mprl_msgs::Action::ConstPtr &actionmsg)
{
  Vector action;
  toVector(actionmsg->action, action);
  
  if (action.size() != action_dims_)
    ERROR("Action received through ROS (size " << action.size() << ") is not compatible with environment (size " << action_dims_ << ")");

  action_writer_ += action;
}
