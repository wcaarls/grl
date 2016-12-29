/** \file leo_preprogrammed.h
 * \brief Leo preprogrammed agent header file.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2016-07-25
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Ivan Koryakovskiy
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

#ifndef GRL_LEO_PREPROGRAMMED_AGENT_H_
#define GRL_LEO_PREPROGRAMMED_AGENT_H_

#include <grl/agent.h>
#include <grl/samplers/random_generator.h>

namespace grl
{

/// Agent.
class LeoPreprogrammedAgent : public Agent
{
  public:
    TYPEINFO("agent/leo_preprogrammed", "Leo preprogrammed agent")

  protected:
    double time_, epsilon_;
    Vector min_, max_;
    int swing_leg_prev_touch_;
    double mSwingTime, mPreProgEarlySwingTime, mPreProgTorsoAngle, mPreProgShoulderAngle,
           mPreProgHipAngle, mPreProgAnkleAngle, mPreProgStanceKneeAngle;

    RandomGenerator *rand_gen_;

  public:
    LeoPreprogrammedAgent() :
      time_(0.), epsilon_(0.05), swing_leg_prev_touch_(0), mSwingTime(0.), mPreProgEarlySwingTime(0.184), mPreProgTorsoAngle(-0.09), mPreProgShoulderAngle(-0.262),
      mPreProgHipAngle(0.680), mPreProgAnkleAngle(0.065), mPreProgStanceKneeAngle(0.)
    { }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual LeoPreprogrammedAgent *clone() const;
    virtual TransitionType start(const Vector &obs, Vector *action);
    virtual TransitionType step(double tau, const Vector &obs, double reward, Vector *action);
    virtual void end(double tau, const Vector &obs, double reward);

    // own
    virtual void auto_actuate(const Vector &obs, Vector *action);
    virtual void autoActuateArm(const Vector &obs, double &shoulderVoltage);
    virtual void autoActuateAnkles_FixedPos(const Vector &obs, double &stanceAnkleVoltage, double &swingAnkleVoltage);
    virtual void autoActuateKnees(const Vector &obs, double &stanceKneeVoltage, double &swingKneeVoltage);
    virtual void autoActuateHips2(const Vector &obs, double &stanceHipVoltage, double &swingHipVoltage);
};

}

#endif /* GRL_LEO_PREPROGRAMMED_AGENT_H_ */
