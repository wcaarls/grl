/*
 * agent_node.h
 *
 *  Created on: Nov 16, 2011
 *      Author: wcaarls
 */

#ifndef MPRL_AGENT_NODE_H_
#define MPRL_AGENT_NODE_H_

#include <boost/thread.hpp>

#include <ros/ros.h>

#include <mprl_msgs/Action.h>
#include <mprl_msgs/StateReward.h>
#include <mprl_msgs/EnvDescription.h>
#include <mprl_msgs/Configuration.h>

#include <mprl_common/mprl.h>

namespace mprl
{
  class AgentNode
  {
    public:
      enum Mentorship { msNone, msMentor, msPupil };
  
    protected:
      enum AgentState { asUninitialized, asQueued, asInitialized, asRunning };

      ros::NodeHandle nh_agent_, nh_env_;
      Agent *agent_;
      Configuration config_;
      AgentState state_;
      Mentorship mentorship_;
      SyncVariable<Action> suggested_action_;

      mprl_msgs::EnvDescription::ConstPtr description_;
      size_t steps_, config_num_;
      size_t action_dims_, state_dims_;

      ros::Subscriber desc_sub_;
      ros::Publisher  action_pub_;
      ros::Subscriber action_sub_;
      ros::Subscriber state_sub_;
      ros::Subscriber message_sub_;

      mprl_msgs::StateReward::ConstPtr queued_state_;

      boost::recursive_mutex mutex_;

      void callbackState(const mprl_msgs::StateReward::ConstPtr &msg);
      void callbackAction(const mprl_msgs::Action::ConstPtr &msg);
      void callbackDesc(const mprl_msgs::EnvDescription::ConstPtr &msg);
      void callbackMessage(const mprl_msgs::Configuration::ConstPtr &msg);
      void loadPlugins(const char *pattern);

    public:
      AgentNode() : nh_agent_("rl_agent"), nh_env_("rl_env"), agent_(NULL), state_(asUninitialized), config_num_(0), action_dims_(1) { }
      ~AgentNode()
      {
        delete agent_;
        ros::shutdown();
      }

      void init(Agent *agent, const Configuration *config=NULL);
      void cleanup();
      void spin();
  };
}

#endif /* MPRL_AGENT_NODE_H_ */
