/*
 * env_node.h
 *
 *  Created on: Nov 11, 2011
 *      Author: wcaarls
 */

#ifndef MPRL_ENV_NODE_H_
#define MPRL_ENV_NODE_H_

#include <ros/ros.h>

#include <mprl_msgs/Action.h>
#include <mprl_msgs/StateReward.h>
#include <mprl_msgs/EnvDescription.h>
#include <mprl_msgs/Configuration.h>
#include <std_msgs/Float32.h>

#include <mprl_common/mprl.h>

namespace mprl
{
  class EnvironmentNode
  {
    protected:
      Environment *environment_;

      ros::NodeHandle nh_agent_, nh_env_;
      ros::Rate *action_rate_;

      ros::Publisher  desc_pub_;
      ros::Subscriber  rate_sub_;
      ros::Subscriber action_sub_;
      ros::Publisher  state_pub_;
      ros::Subscriber message_sub_;
      
      size_t state_dims_, action_dims_;

      void callbackAction(const mprl_msgs::Action::ConstPtr &msg);
      void callbackMessage(const mprl_msgs::Configuration::ConstPtr &msg);
      
      void start();

    public:
      EnvironmentNode() : environment_(NULL), nh_agent_("rl_agent"), nh_env_("rl_env"), action_rate_(NULL) { }
      ~EnvironmentNode()
      {
        ros::shutdown();
        ros::waitForShutdown();
        delete environment_;
      }

      void init(Environment *environment, const Configuration *config=NULL);
      void spin();
  };
}

#endif /* MPRL_ENV_NODE_H_ */
