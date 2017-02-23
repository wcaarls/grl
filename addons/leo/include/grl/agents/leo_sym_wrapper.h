/** \file leo_sym_wrapper.h
 * \brief Leo agent wrapper header file.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2017-02-07
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

#ifndef GRL_LEO_WALKING_WRAPPER_AGENT_H_
#define GRL_LEO_WALKING_WRAPPER_AGENT_H_

#include <grl/agent.h>
#include <grl/signal.h>

namespace grl
{

/// Temporal difference learning agent for leo.
class LeoSymWrapperAgent : public Agent
{
  public:
    TYPEINFO("agent/leo/sym_wrapper", "Leo agent that symmetrically wraps angles and controls")

  protected: 
    Agent *agent_;
    VectorSignal *sub_ic_signal_;
    double preProgShoulderAngle_, preProgStanceKneeAngle_, preProgAnkleAngle_;

  public:
    LeoSymWrapperAgent() : agent_(NULL), sub_ic_signal_(NULL), preProgShoulderAngle_(-0.261799387799149), preProgStanceKneeAngle_(0), preProgAnkleAngle_(0.0649262481741891) { }

    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual void start(const Observation &obs, Action *action);
    virtual void step(double tau, const Observation &obs, double reward, Action *action);
    virtual void end(double tau, const Observation &obs, double reward);

  protected:
    virtual int stanceLegLeft() const;
    virtual void parseStateForAgent(const Observation &obs, Observation *obs_agent, int stl) const;
    virtual void parseActionForEnvironment(const Action &act_agent, const Observation &obs, Action *action, int stl) const;
    virtual double autoActuateArm(double armObs) const;
    virtual double autoActuateKnees(double stanceKneeObs) const;
    virtual void autoActuateAnkles_FixedPos(double leftAnkleObs, double *leftAnkleAction, double rightAnkleObs, double *rightAnkleAction) const;

};

}

#endif /* GRL_LEO_WALKING_WRAPPER_AGENT_H_ */
