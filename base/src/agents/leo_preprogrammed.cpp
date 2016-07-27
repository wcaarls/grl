/** \file leo_preprogrammed.cpp
 * \brief Leo preprogrammed agent source file.
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

#include <grl/agents/leo_preprogrammed.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoPreprogrammedAgent)

// Enumeration comes from the way state is peovided by leosim module
enum LeoJointStateIndexies {
      sTorsoAngle,
      sShoulderAngle,
      sHipSwingAngle,
      sHipStanceAngle,
      sKneeSwingAngle,
      sKneeStanceAngle,
      sAnkleSwingAngle,
      sAnkleStanceAngle,
      sTorsoAngleRate,
      sShoulderAngleRate,
      sHipSwingAngleRate,
      sHipStanceAngleRate,
      sKneeSwingAngleRate,
      sKneeStanceAngleRate,
      sAnkleSwingAngleRate,
      sAnkleStanceAngleRate,
      sToeSwing,
      sHeelSwing,
      sToeStance,
      sHeelStance
};

//enum LeoJointActionIndexies { aShoulder, aHipSwing, aHipStance, aKneeSwing, aKneeStance, aAnkleSwing, aAnkleStance };
// Right leg is assumed to be a stance leg in the configuration file
enum LeoJointActionIndexies { aShoulder, aHipStance, aHipSwing, aKneeStance, aKneeSwing, aAnkleStance, aAnkleSwing };

void LeoPreprogrammedAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("output_min", "vector.action_min", "Lower limit on outputs", min_, CRP::System));
  config->push_back(CRP("output_max", "vector.action_max", "Upper limit on outputs", max_, CRP::System));
}

void LeoPreprogrammedAgent::configure(Configuration &config)
{
  min_ = config["output_min"].v();
  max_ = config["output_max"].v();
}

void LeoPreprogrammedAgent::reconfigure(const Configuration &config)
{
}

LeoPreprogrammedAgent *LeoPreprogrammedAgent::clone() const
{
  LeoPreprogrammedAgent *agent = new LeoPreprogrammedAgent();
  return agent;
}

void LeoPreprogrammedAgent::start(const Vector &obs, Vector *action)
{
  time_ = mSwingTime = 0.;
  auto_actuate(obs, action);
}

void LeoPreprogrammedAgent::step(double tau, const Vector &obs, double reward, Vector *action)
{
  time_ += tau;
  mSwingTime += tau;
  auto_actuate(obs, action);
}

void LeoPreprogrammedAgent::end(double tau, const Vector &obs, double reward)
{
}

void LeoPreprogrammedAgent::auto_actuate(const Vector &obs, Vector *action)
{
  std::cout << obs << std::endl;

  // restart time at touch down
  int swing_leg_touch = (obs[sToeSwing] || obs[sHeelSwing]) ? 1:0;
  if ((swing_leg_prev_touch_ != swing_leg_touch) && (swing_leg_touch == 1))
    mSwingTime = 0;
  swing_leg_prev_touch_ = swing_leg_touch;

  // auto actuate all joints
  double shoulderVoltage, stanceHipVoltage, swingHipVoltage, stanceKneeVoltage, swingKneeVoltage, stanceAnkleVoltage, swingAnkleVoltage;
  autoActuateAnkles_FixedPos(obs, stanceAnkleVoltage, swingAnkleVoltage);
  autoActuateArm(obs, shoulderVoltage);
  autoActuateKnees(obs, stanceKneeVoltage, swingKneeVoltage);
  autoActuateHips2(obs, stanceHipVoltage, swingHipVoltage);

  // Disturb the standard controller, only if learning is enabled (not for test runs)
//	if (mIsObserving && mLearningEnabled)
//		if (gRanrotB.Random() < mPreProgExploreRate)
//			mAgentAction.randomizeUniform(&gRanrotB);

  action->resize(min_.size());
  (*action)[aShoulder] = shoulderVoltage;
  (*action)[aHipStance] = stanceHipVoltage;
  (*action)[aHipSwing] = swingHipVoltage;
  (*action)[aKneeStance] = stanceKneeVoltage;
  (*action)[aKneeSwing] = swingKneeVoltage;
  (*action)[aAnkleStance] = stanceAnkleVoltage;
  (*action)[aAnkleSwing] = swingAnkleVoltage;

  std::cout << *action << std::endl;
}

void LeoPreprogrammedAgent::autoActuateAnkles_FixedPos(const Vector &obs, double &stanceAnkleVoltage, double &swingAnkleVoltage)
{
  const double torqueToVoltage	= 14.0/3.3;

  double K					= 10.0*torqueToVoltage;
  double D					= 0;//0.00992*193.0*1.1;
  stanceAnkleVoltage	= K*(mPreProgAnkleAngle - obs[sAnkleStanceAngle]) + D*obs[sAnkleStanceAngleRate];
  swingAnkleVoltage		= K*(mPreProgAnkleAngle - obs[sAnkleSwingAngle]) + D*obs[sAnkleSwingAngleRate];
}

void LeoPreprogrammedAgent::autoActuateArm(const Vector &obs, double &shoulderVoltage)
{
  const double torqueToVoltage	= 14.0/3.3;

  double armTorque = 5.0*(mPreProgShoulderAngle - obs[sShoulderAngle]);
  shoulderVoltage = torqueToVoltage*armTorque;
}

void LeoPreprogrammedAgent::autoActuateKnees(const Vector &obs, double &stanceKneeVoltage, double &swingKneeVoltage)
{
  // The stance knee contains a weak controller to remain stretched
  const double torqueToVoltage = 14.0/3.3;
  double stanceKneeTorque = 5.0*(mPreProgStanceKneeAngle - obs[sKneeStanceAngle]);
  stanceKneeVoltage = torqueToVoltage*stanceKneeTorque;

  // swing knee
  if (mSwingTime < mPreProgEarlySwingTime)
  {
    // Early swing
    swingKneeVoltage		= -14.0;	// Most probably clipped due to thermal guarantees
  }
  else
  {
    // Late swing
    swingKneeVoltage		= 65.0*(mPreProgStanceKneeAngle - obs[sKneeSwingAngle]);
  }
  swingKneeVoltage = fmax(fmin(swingKneeVoltage, max_[aKneeSwing]), min_[aKneeSwing]);
}

void LeoPreprogrammedAgent::autoActuateHips2(const Vector &obs, double &stanceHipVoltage, double &swingHipVoltage)
{
  // The "torque" here is not actually torque, but a leftover from the "endless turn mode" control from dynamixels, which is actually voltage control
  const double torqueToVoltage	= 14.0/3.3;
  // Hip: control the inter hip angle to a fixed angle
  double interHipAngleTorque	= 4.0*(mPreProgHipAngle - (obs[sHipSwingAngle] - obs[sHipStanceAngle]));

  double stanceHipTorque	= (-0.20)*interHipAngleTorque;
  double swingHipTorque		= interHipAngleTorque;

  //if (mStanceFootContact)
  {
    // Torque to keep the upper body up right
     double stanceTorque = -14.0*(mPreProgTorsoAngle - obs[sTorsoAngle]);
    stanceHipTorque			+= stanceTorque;
  }

  stanceHipVoltage = torqueToVoltage * stanceHipTorque;
  swingHipVoltage = torqueToVoltage * swingHipTorque;

  stanceHipVoltage = fmax(fmin(stanceHipVoltage, max_[aHipStance]), min_[aHipStance]);
  swingHipVoltage  = fmax(fmin(swingHipVoltage, max_[aHipSwing]), min_[aHipSwing]);
}

