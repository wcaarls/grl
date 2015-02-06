/*
 * agent_node.cpp
 *
 *  Created on: Nov 16, 2011
 *      Author: wcaarls
 */

#include <libgen.h>
#include <glob.h>
#include <dlfcn.h>

#include <boost/thread.hpp>

#include <mprl_ros/mprl_ros.h>
#include <mprl_ros/agent_node.h>

using namespace mprl;

void AgentNode::callbackDesc(const mprl_msgs::EnvDescription::ConstPtr &descmsg)
{
  boost::lock_guard<boost::recursive_mutex> guard(mutex_);

  description_ = descmsg;
  
  if (state_ == asInitialized || state_ == asRunning)
  {
    ROS_WARN("Reinitializing agent");
    agent_->cleanup();
    state_ = asUninitialized;
  }
  else
    ROS_INFO("Initializing agent");

  Configuration config = config_;

  config.set("observation_dims", descmsg->observation_min.size());
  config.set("action_dims", descmsg->action_min.size());

  Vector v;

  toVector(descmsg->observation_min, v);
  state_dims_ = v.size();
  config.set("observation_min", v);
  toVector(descmsg->observation_max, v);
  config.set("observation_max", v);
  toVector(descmsg->action_min, v);
  action_dims_ = v.size();
  config.set("action_min", v);
  toVector(descmsg->action_max, v);
  config.set("action_max", v);

  config.set("reward_min", descmsg->reward_min);
  config.set("reward_max", descmsg->reward_max);

  config.set("stochastic", descmsg->stochastic);
  config.set("episodic", descmsg->episodic);
  config.set("title", descmsg->title);

  std::string path;

  if (!config.get("config_path", path))
  {
    char buf[PATH_MAX] = { 0 };
    if (readlink("/proc/self/exe", buf, PATH_MAX) < 0)
    ROS_WARN("Couldn't locate executable");
    path = dirname(buf);
    path = path + "/../cfg";
    config.set("config_path", path);
  }

  std::ostringstream ss;
  ss << path << "/" << descmsg->title;
  if (config_num_ > 0)
    ss << "-" << config_num_;
  ss << ".yaml";

  std::string str = ss.str();

  if (!config.loadYAML(str, true))
    ROS_WARN_STREAM("Couldn't load configuration " << str);
  else
    ROS_INFO_STREAM("Loaded configuration " << str);

  std::string output_file;
  config.get("output_file", output_file, "");

  if (output_file != "")
  {
    // Copy configuration
    std::ifstream f1 (str.c_str(), std::fstream::binary);
    std::ofstream f2 ((output_file + ".yaml").c_str(), std::fstream::trunc|std::fstream::binary);
    f2 << f1.rdbuf();
  }

  steps_ = 0;
  config_num_++;

  agent_->init(config);

  if (state_ == asQueued)
  {
    state_ = asInitialized;

    ROS_INFO("Processing queued state");
    callbackState(queued_state_);
    queued_state_.reset();
  }
  else
    state_ = asInitialized;
}

void AgentNode::callbackState(const mprl_msgs::StateReward::ConstPtr &statemsg)
{
  boost::lock_guard<boost::recursive_mutex> guard(mutex_);

  if (state_ == asQueued)
    ROS_WARN("Overwriting queued state.");

  if (state_ == asUninitialized || state_ == asQueued)
  {
    ROS_INFO("Queueing state message while waiting for environment description.");
    queued_state_ = statemsg;

    state_ = asQueued;
    return;
  }

  Observation obs;
  Action action;
  
  // If we're a pupil, wait for action suggested
  // by mentor (unless the state is absorbing)
  if (mentorship_ == msPupil && !statemsg->absorbing)
    action = suggested_action_.get();

  toVector(statemsg->state, obs);
  
  ROS_ASSERT(obs.size() == state_dims_);

  if (statemsg->terminal)
  {
    ROS_DEBUG("Episode ended");

    if (statemsg->absorbing)
      agent_->end(statemsg->reward);
    else
    {
      agent_->step(statemsg->reward, obs, action);
      steps_++;
    }

    state_ = asInitialized;
    
    // If we're a mentor, suggest an action even if the
    // epsiode ended (unless the state is absorbing)
    if (mentorship_ != msMentor || statemsg->absorbing)
      return;
  }
  else if (state_ == asRunning)
  {
    // if action is provided for pupil, it will be overwritten below.
    agent_->step(statemsg->reward, obs, action);
    steps_++;
  }
  else
  {
    ROS_DEBUG("Starting new episode");
    if (!agent_->start(obs, action))
    {
      callbackDesc(description_);
      agent_->start(obs, action);
    }
    state_ = asRunning;
  }

  mprl_msgs::Action actionmsg;

  ROS_ASSERT(action.size() == action_dims_);
  
  fromVector(action, actionmsg.action);

  action_pub_.publish(actionmsg);
}

void AgentNode::callbackAction(const mprl_msgs::Action::ConstPtr &msg)
{
  // Note absence of guard. That means callbackState can wait for this callback.
  Action action;
  
  toVector(msg->action, action);

  ROS_ASSERT(action.size() == action_dims_);

  suggested_action_.set(action);
}

void AgentNode::callbackMessage(const mprl_msgs::Configuration::ConstPtr &msg)
{
  boost::lock_guard<boost::recursive_mutex> guard(mutex_);

  Configuration config;
  
  config.set("type", msg->type);
  
  for (size_t ii=0; ii < msg->options.size(); ++ii)
    config.set(msg->options[ii].key, msg->options[ii].value);
    
  Configuration result;
  agent_->message(config, result);
}

void AgentNode::loadPlugins(const char *pattern)
{
  glob_t globbuf;
  
  ROS_DEBUG("Looking for plugins in '%s'", pattern);
  
  glob(pattern, 0, NULL, &globbuf);
  for (int ii=0; ii < globbuf.gl_pathc; ++ii)
  {
    ROS_INFO("Loading plugin '%s'", globbuf.gl_pathv[ii]);
    if (!dlopen(globbuf.gl_pathv[ii], RTLD_NOW|RTLD_LOCAL))
      ROS_WARN("Error loading plugin '%s': %s", globbuf.gl_pathv[ii], dlerror());
  }
}

void AgentNode::init(Agent *agent, const Configuration *config)
{
  agent_ = agent;
  if (config) config_ = *config;
  
  // Load plugins
  char buf[PATH_MAX] = { 0 };
  if (readlink("/proc/self/exe", buf, PATH_MAX) > 0)
  {
    char patbuf[PATH_MAX];
    
    strcpy(patbuf, buf);
    sprintf(patbuf, "%s/../libenv_*.so", dirname(patbuf));
    loadPlugins(patbuf);
    
    strcpy(patbuf, buf);
    sprintf(patbuf, "%s/../libma_*.so", dirname(patbuf));
    loadPlugins(patbuf);
  }
  else
    ROS_WARN("Couldn't locate executable");

  state_ = asUninitialized;

  int mentorship;
  config_.get("mentorship", mentorship, 0);
  mentorship_ = (Mentorship)mentorship;
  
  if (mentorship_ == msMentor)
  {
    // Publish action as suggestion
    action_pub_ = nh_agent_.advertise<mprl_msgs::Action> ("rl_suggested_action", 10, true);
    
    ROS_INFO("Agent running in mentor mode");
  }
  else
  {
    // Publish action as command
    action_pub_ = nh_agent_.advertise<mprl_msgs::Action> ("rl_action", 10, true);
    
    if (mentorship_ == msPupil)
    {
      // Subscribe to suggested actions
      action_sub_ = nh_agent_.subscribe("rl_suggested_action", 10, &AgentNode::callbackAction, this);

      ROS_INFO("Agent running in pupil mode");
    }
  }
    
  state_sub_ = nh_env_.subscribe("rl_state_reward", 10, &AgentNode::callbackState, this);
  desc_sub_ = nh_env_.subscribe("rl_env_description", 10, &AgentNode::callbackDesc, this);
  message_sub_ = nh_agent_.subscribe("rl_message", 10, &AgentNode::callbackMessage, this);
}

void AgentNode::cleanup()
{
  suggested_action_.terminate();
}

void AgentNode::spin()
{
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
}
