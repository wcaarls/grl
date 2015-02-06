/*
 * env_node.cpp
 *
 *  Created on: Nov 11, 2011
 *      Author: wcaarls
 */

#include <mprl_ros/env_node.h>

using namespace mprl;

void EnvironmentNode::callbackAction(const mprl_msgs::Action::ConstPtr &actionmsg)
{
  Action action;
  toVector(actionmsg->action, action);
  
  ROS_ASSERT(action.size() == action_dims_);

  ObservationRewardTerminal ort;
  environment_->step(action, ort);
  
  ROS_ASSERT(ort.observation.size() == state_dims_);

  if (action_rate_)
    action_rate_->sleep();

  mprl_msgs::StateReward statemsg;
  statemsg.state.resize(ort.observation.size());
  for (size_t ii=0; ii < ort.observation.size(); ++ii)
    statemsg.state[ii] = ort.observation[ii];
  statemsg.reward = ort.reward;
  statemsg.terminal = ort.terminal;
  statemsg.absorbing = ort.absorbing;
  state_pub_.publish(statemsg);

  if (ort.terminal)
    start();
}

void EnvironmentNode::callbackMessage(const mprl_msgs::Configuration::ConstPtr &msg)
{
  Configuration config;
  for (size_t ii=0; ii < msg->options.size(); ++ii)
    config.set(msg->options[ii].key, msg->options[ii].value);

  double rate;    
  if (config.get("rate", rate))
  {
    safe_delete(&action_rate_);
    if (rate)
      action_rate_ = new ros::Rate(rate);
  }

  Configuration result;
  environment_->message(config, result);
}

void EnvironmentNode::start()
{
  Observation observation;
  environment_->start(observation);

  ROS_ASSERT(observation.size() == state_dims_);

  if (action_rate_)
    action_rate_->reset();

  mprl_msgs::StateReward statemsg;
  statemsg.state.resize(observation.size());
  for (size_t ii=0; ii < observation.size(); ++ii)
    statemsg.state[ii] = observation[ii];
  statemsg.reward = 0;
  statemsg.terminal = false;
  statemsg.absorbing = false;

  state_pub_.publish(statemsg);
}

void EnvironmentNode::init(Environment *environment, const Configuration *config)
{
  environment_ = environment;

  action_sub_ = nh_agent_.subscribe("rl_action", 10, &EnvironmentNode::callbackAction, this);
  message_sub_ =  nh_env_.subscribe("rl_message", 10, &EnvironmentNode::callbackMessage, this);
  state_pub_ = nh_env_.advertise<mprl_msgs::StateReward>("rl_state_reward", 10, true);
  desc_pub_ = nh_env_.advertise<mprl_msgs::EnvDescription>("rl_env_description", 10, true);

  Configuration task_config, task_spec;

  if (config)
  {
    double rate;
    config->get("rate", rate, 0.0);
    if (rate)
      action_rate_ = new ros::Rate(rate);

    task_config = *config;
  }

  environment_->init(task_config, task_spec);

  mprl_msgs::EnvDescription descmsg;

  Vector v;

  v = task_spec["observation_min"];
  state_dims_ = v.size();
  fromVector(v, descmsg.observation_min);
  v = task_spec["observation_max"];
  fromVector(v, descmsg.observation_max);
  v = task_spec["action_min"];
  action_dims_ = v.size();
  fromVector(v, descmsg.action_min);
  v = task_spec["action_max"];
  fromVector(v, descmsg.action_max);

  descmsg.reward_min = task_spec["reward_min"];
  descmsg.reward_max = task_spec["reward_max"];

  descmsg.stochastic = task_spec["stochastic"];
  descmsg.episodic = task_spec["episodic"];
  task_spec.get("title", descmsg.title);

  desc_pub_.publish(descmsg);

  // Give agent some time to reinitialize
  usleep(100000);

  start();
}

void EnvironmentNode::spin()
{
  if (ros::ok())
  {
    ROS_INFO("Spinning");
    ros::spin();
  }
}
