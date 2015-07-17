/** \file ros.h
 * \brief ROS environment header file.
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

#ifndef GRL_ROS_ENVIRONMENT_H_
#define GRL_ROS_ENVIRONMENT_H_

#include <ros/ros.h>
#include <mprl_msgs/Action.h>
#include <mprl_msgs/StateReward.h>
#include <mprl_msgs/EnvDescription.h>

#include <itc/queue.h>

#include <grl/environment.h>

namespace grl
{

/// External environment interfaced through ROS. 
class ROSEnvironment : public Environment
{
  public:
    TYPEINFO("environment/ros", "External environment interfaced through ROS")

  protected:
    itc::Queue<mprl_msgs::EnvDescription> desc_queue_;
    itc::QueueReader<mprl_msgs::EnvDescription> desc_reader_;
    itc::QueueWriter<mprl_msgs::EnvDescription> desc_writer_;
    itc::Queue<mprl_msgs::StateReward> state_queue_;
    itc::QueueReader<mprl_msgs::StateReward> state_reader_;
    itc::QueueWriter<mprl_msgs::StateReward> state_writer_;
    bool running_;
    
    std::string node_, args_;
    size_t state_dims_;
    double tau_;
  
    ros::NodeHandle *nh_agent_, *nh_env_;
    ros::Subscriber desc_sub_, state_sub_;
    ros::Publisher  action_pub_;
    ros::AsyncSpinner *spinner_;
    
  public:
    ROSEnvironment() : running_(false), state_dims_(0), tau_(0.05), nh_agent_(NULL), nh_env_(NULL), spinner_(NULL)
    {
      desc_reader_ = desc_queue_.addReader();
      desc_writer_ = desc_queue_.getWriter();
      state_reader_ = state_queue_.addReader();
      state_writer_ = state_queue_.getWriter();
    }
    
    ~ROSEnvironment()
    {
      safe_delete(&nh_agent_);
      safe_delete(&nh_env_);
      safe_delete(&spinner_);
    }
    
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
      
    // From Environment
    virtual ROSEnvironment *clone() const;
    virtual void start(int test, Vector *obs);
    virtual double step(const Vector &action, Vector *obs, double *reward, int *terminal);
  
  protected:
    void callbackDesc(const mprl_msgs::EnvDescription::ConstPtr &descmsg);
    void callbackState(const mprl_msgs::StateReward::ConstPtr &statemsg);
};

}

#endif /* GRL_ROS_ENVIRONMENT_H_ */
