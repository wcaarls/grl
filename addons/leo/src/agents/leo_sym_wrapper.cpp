/** \file leo_sym_wrapper.cpp
 * \brief Leo agent wrapper source file.
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

#include <grl/agents/leo_sym_wrapper.h>
#include <leo.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoSymWrapperAgent)

enum LeoXmlStateVar
{
  xsvTorsoAngle,
  xsvLeftArmAngle,
  xsvRightHipAngle,
  xsvLeftHipAngle,
  xsvRightKneeAngle,
  xsvLeftKneeAngle,
  xsvRightAnkleAngle,
  xsvLeftAnkleAngle,
  xsvTorsoAngleRate,
  xsvLeftArmAngleRate,
  xsvRightHipAngleRate,
  xsvLeftHipAngleRate,
  xsvRightKneeAngleRate,
  xsvLeftKneeAngleRate,
  xsvRightAnkleAngleRate,
  xsvLeftAnkleAngleRate
};

void LeoSymWrapperAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("agent", "agent", "Target agent with reduced state-action space due to symmetry", agent_));
  config->push_back(CRP("sub_ic_signal", "signal/vector", "Publisher of the initialization and contact signal", sub_ic_signal_));
}

void LeoSymWrapperAgent::configure(Configuration &config)
{
  agent_ = (Agent*)config["agent"].ptr();
  sub_ic_signal_ = (VectorSignal*)config["sub_ic_signal"].ptr();
}

void LeoSymWrapperAgent::reconfigure(const Configuration &config)
{
}

void LeoSymWrapperAgent::start(const Observation &obs, Action *action)
{
  Observation obs_agent = ConstantVector(10, 0);
  Action act_agent = ConstantVector(3, 0);

  action->v.resize(7);

  int stl = stanceLegLeft();
  parseStateForAgent(obs, &obs_agent, stl);
  agent_->start(obs_agent, &act_agent);
  parseActionForEnvironment(act_agent, obs, action, stl);
}

void LeoSymWrapperAgent::step(double tau, const Observation &obs, double reward, Action *action)
{
  Observation obs_agent = ConstantVector(10, 0);
  Action act_agent = ConstantVector(3, 0);

  int stl = stanceLegLeft();
  parseStateForAgent(obs, &obs_agent, stl);
  agent_->step(tau, obs_agent, reward, &act_agent);
  parseActionForEnvironment(act_agent, obs, action, stl);
}

void LeoSymWrapperAgent::end(double tau, const Observation &obs, double reward)
{
  Observation obs_agent = ConstantVector(10, 0);

  int stl = stanceLegLeft();
  parseStateForAgent(obs, &obs_agent, stl);
  agent_->end(tau, obs_agent, reward);
}

int LeoSymWrapperAgent::stanceLegLeft() const
{
  double stl = 0;
  Vector signal = sub_ic_signal_->get();
  if ((int)signal[0] & lstStanceLeft)
    stl = 1;

  // update signal because of symmetrical state
  if ((int)signal[0] & lstSwlTouchDown)
  {
    Vector ti_actuator = VectorConstructor(-1, 1, 0, 2, 2, -1, -1);
    Vector ti_actuator_sym = VectorConstructor(-1, 0, 1, 2, 2, -1, -1);

    if (stl)
      signal << signal[0], ti_actuator, ti_actuator_sym;
    else
      signal << signal[0], ti_actuator_sym, ti_actuator;
    sub_ic_signal_->set(signal);

    INFO("LeoSymWrapperAgent : TouchDown");
  }

  return stl;
}

void LeoSymWrapperAgent::parseStateForAgent(const Observation &obs, Observation *obs_agent, int stl) const
{
  // environment
  //   observe: torso_boom, shoulder, hipright, hipleft, kneeright, kneeleft, ankleright, ankleleft
  //   actuate: shoulder, hipright, hipleft, kneeright, kneeleft, ankleright, ankleleft

  CRAWL(obs);

  (*obs_agent)[0] = obs[LeoXmlStateVar::xsvTorsoAngle];
  (*obs_agent)[5] = obs[LeoXmlStateVar::xsvTorsoAngleRate];

  if (stl)
  {
    (*obs_agent)[1] = obs[LeoXmlStateVar::xsvRightHipAngle];
    (*obs_agent)[2] = obs[LeoXmlStateVar::xsvLeftHipAngle];
    (*obs_agent)[3] = obs[LeoXmlStateVar::xsvRightKneeAngle];
    (*obs_agent)[4] = obs[LeoXmlStateVar::xsvLeftKneeAngle];

    (*obs_agent)[6] = obs[LeoXmlStateVar::xsvRightHipAngleRate];
    (*obs_agent)[7] = obs[LeoXmlStateVar::xsvLeftHipAngleRate];
    (*obs_agent)[8] = obs[LeoXmlStateVar::xsvRightKneeAngleRate];
    (*obs_agent)[9] = obs[LeoXmlStateVar::xsvLeftKneeAngleRate];
  }
  else
  {
    (*obs_agent)[2] = obs[LeoXmlStateVar::xsvRightHipAngle];
    (*obs_agent)[1] = obs[LeoXmlStateVar::xsvLeftHipAngle];
    (*obs_agent)[4] = obs[LeoXmlStateVar::xsvRightKneeAngle];
    (*obs_agent)[3] = obs[LeoXmlStateVar::xsvLeftKneeAngle];

    (*obs_agent)[7] = obs[LeoXmlStateVar::xsvRightHipAngleRate];
    (*obs_agent)[6] = obs[LeoXmlStateVar::xsvLeftHipAngleRate];
    (*obs_agent)[9] = obs[LeoXmlStateVar::xsvRightKneeAngleRate];
    (*obs_agent)[8] = obs[LeoXmlStateVar::xsvLeftKneeAngleRate];
  }

  obs_agent->absorbing = obs.absorbing;

  TRACE(*obs_agent);
}

void LeoSymWrapperAgent::parseActionForEnvironment(const Action &act_agent, const Observation &obs, Action *action, int stl) const
{
  // agent
  //   observe: torso_boom, hipright, hipleft, kneeright, kneeleft
  //   actuate: hipright, hipleft, stanceknee

  TRACE(act_agent);

  (*action)[CLeoBhBase::avLeftArmAction] = autoActuateArm(obs[LeoXmlStateVar::xsvLeftArmAngle]);

  if (stl)
  {
    (*action)[CLeoBhBase::avRightHipAction]  = act_agent[0];
    (*action)[CLeoBhBase::avLeftHipAction]   = act_agent[1];
    (*action)[CLeoBhBase::avRightKneeAction] = act_agent[2];
    (*action)[CLeoBhBase::avLeftKneeAction]  = autoActuateKnees(obs[LeoXmlStateVar::xsvLeftKneeAngle]); // auto actuate stance leg, which is left now
  }
  else
  {
    (*action)[CLeoBhBase::avRightHipAction]  = act_agent[1];
    (*action)[CLeoBhBase::avLeftHipAction]   = act_agent[0];
    (*action)[CLeoBhBase::avRightKneeAction] = autoActuateKnees(obs[LeoXmlStateVar::xsvRightKneeAngle]); // auto actuate stance leg, which is right now
    (*action)[CLeoBhBase::avLeftKneeAction]  = act_agent[2];
  }

  double leftAnkleAction, rightAnkleAction;
  autoActuateAnkles_FixedPos(obs[LeoXmlStateVar::xsvLeftAnkleAngle],  &leftAnkleAction,
                             obs[LeoXmlStateVar::xsvRightAnkleAngle], &rightAnkleAction);
  (*action)[CLeoBhBase::avLeftAnkleAction] = leftAnkleAction;
  (*action)[CLeoBhBase::avRightAnkleAction] = rightAnkleAction;

  action->type = act_agent.type;

  CRAWL(*action);
}

double LeoSymWrapperAgent::autoActuateArm(double armObs) const
{
  double armTorque = 5.0*(preProgShoulderAngle_ - armObs);
  const double torqueToVoltage  = 14.0/3.3;
  return torqueToVoltage*armTorque;
}

double LeoSymWrapperAgent::autoActuateKnees(double stanceKneeObs) const
{
  double kneeStanceTorque = 5.0*(preProgStanceKneeAngle_ - stanceKneeObs);
  const double torqueToVoltage = 14.0/3.3;
  return torqueToVoltage*kneeStanceTorque;
}

void LeoSymWrapperAgent::autoActuateAnkles_FixedPos(double leftAnkleObs, double *leftAnkleAction, double rightAnkleObs, double *rightAnkleAction) const
{
  double K = 10.0;
  double leftAnkleTorque    = K*(preProgAnkleAngle_ - leftAnkleObs);
  double rightAnkleTorque   = K*(preProgAnkleAngle_ - rightAnkleObs);

  const double torqueToVoltage  = 14.0/3.3;
  *leftAnkleAction = leftAnkleTorque*torqueToVoltage;
  *rightAnkleAction = rightAnkleTorque*torqueToVoltage;
}
