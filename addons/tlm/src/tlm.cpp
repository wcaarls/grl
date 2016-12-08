/** \file tlm.cpp
 * \brief Two-link manipulator environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-12
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

#include <eigen3/Eigen/Eigen>

#include <grl/environments/tlm.h>

using namespace grl;

REGISTER_CONFIGURABLE(TwoLinkManipulatorDynamics)
REGISTER_CONFIGURABLE(TwoLinkManipulatorBalancingTask)
REGISTER_CONFIGURABLE(TwoLinkManipulatorVisualization) 

void TwoLinkManipulatorDynamics::request(ConfigurationRequest *config)
{
}

void TwoLinkManipulatorDynamics::configure(Configuration &config)
{
  double l = 0.4;
  double m1 = 1.25, m2 = 0.8, I1 = 0.066, I2 = 0.043, c1 = 0.2, c2 = 0.2;
  
  p1_ = m1*c1*c1 + m2*l*l + I1;
  p2_ = m2*c2*c2 + I2;
  p3_ = m2*l*c2;
}

void TwoLinkManipulatorDynamics::reconfigure(const Configuration &config)
{
}

void TwoLinkManipulatorDynamics::eom(const Vector &state, const Vector &action, Vector *xd) const
{
  if (state.size() != 5 || action.size() != 2)
    throw Exception("dynamics/tlm requires a task/tlm subclass");
    
  Eigen::Matrix2d M, C;
  Eigen::Vector2d ad(state[siAngleRate1], state[siAngleRate2]);
  Eigen::Vector2d torques(action[0], action[1]);

  double cosa2 = cos(state[siAngle2]);
  double sina2 = sin(state[siAngle2]);
  M << p1_ + p2_ + 2*p3_*cosa2, p2_ + p3_*cosa2, p2_ + p3_*cosa2, p2_;
  C << damping1_ - p3_*state[siAngleRate2]*sina2, -p3_*(state[siAngleRate1] + state[siAngleRate2])*sina2, p3_*state[siAngleRate1]*sina2, damping2_;

  Eigen::Vector2d accels = M.inverse()*(torques.matrix() - (C*ad));

  // Limit velocity to 2*M_PI
  if (state[siAngleRate1] >  2*M_PI) accels[0] = fmin(accels[0], 0);
  if (state[siAngleRate1] < -2*M_PI) accels[0] = fmax(accels[0], 0);
  if (state[siAngleRate2] >  2*M_PI) accels[1] = fmin(accels[1], 0);
  if (state[siAngleRate2] < -2*M_PI) accels[1] = fmax(accels[1], 0);

  *xd = VectorConstructor(state[siAngleRate1], state[siAngleRate2], accels[0], accels[1], 1.);
}

void TwoLinkManipulatorBalancingTask::request(ConfigurationRequest *config)
{
  Task::request(config);
}

void TwoLinkManipulatorBalancingTask::configure(Configuration &config)
{
  config.set("observation_dims", 4);
  config.set("observation_min", VectorConstructor(0.,     0.,     -2*M_PI, -2*M_PI));
  config.set("observation_max", VectorConstructor(2*M_PI, 2*M_PI,  2*M_PI,  2*M_PI));
  config.set("action_dims", 2);
  config.set("action_min", VectorConstructor(-1.5, -1.));
  config.set("action_max", VectorConstructor( 1.5,  1.));
  config.set("reward_min", 0);
  config.set("reward_max", -2.8*M_PI*M_PI);
}

void TwoLinkManipulatorBalancingTask::reconfigure(const Configuration &config)
{
}

void TwoLinkManipulatorBalancingTask::start(int test, Vector *state) const
{
  state->resize(5);
  (*state)[TwoLinkManipulatorDynamics::siAngle1] = RandGen::get()*2*M_PI;
  (*state)[TwoLinkManipulatorDynamics::siAngle2] = RandGen::get()*2*M_PI;
  (*state)[TwoLinkManipulatorDynamics::siAngleRate1] = 0;
  (*state)[TwoLinkManipulatorDynamics::siAngleRate2] = 0;
  (*state)[TwoLinkManipulatorDynamics::siTime] = 0;
}

void TwoLinkManipulatorBalancingTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  if (state.size() != 5)
    throw Exception("task/tlm/balancing requires dynamics/tlm");
    
  obs->resize(4);
  for (size_t ii=0; ii < 2; ++ii)
  {
    (*obs)[ii] = fmod(state[ii]+M_PI, 2*M_PI);
    if ((*obs)[ii] < 0) (*obs)[ii] += 2*M_PI;
  }
  for (size_t ii=2; ii < 4; ++ii)
    (*obs)[ii] = state[ii];

  *terminal = state[TwoLinkManipulatorDynamics::siTime] > 3;
}

void TwoLinkManipulatorBalancingTask::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  if (state.size() != 5 || action.size() != 2 || next.size() != 5)
    throw Exception("task/tlm/balancing requires dynamics/tlm");
    
  double a1 = fmod(fabs(next[TwoLinkManipulatorDynamics::siAngle1]), 2*M_PI),
         a2 = fmod(fabs(next[TwoLinkManipulatorDynamics::siAngle2]), 2*M_PI);
         
  if (a1 > M_PI) a1 -= 2*M_PI;
  if (a2 > M_PI) a2 -= 2*M_PI;
    
  *reward = -1*pow(a1, 2) -0.05*pow(next[TwoLinkManipulatorDynamics::siAngleRate1], 2)
            -1*pow(a2, 2) -0.05*pow(next[TwoLinkManipulatorDynamics::siAngleRate2], 2);
}

bool TwoLinkManipulatorBalancingTask::invert(const Vector &obs, Vector *state) const
{
  *state = VectorConstructor(obs[TwoLinkManipulatorDynamics::siAngle1]-M_PI,
                             obs[TwoLinkManipulatorDynamics::siAngle2]-M_PI,
                             obs[TwoLinkManipulatorDynamics::siAngleRate1],
                             obs[TwoLinkManipulatorDynamics::siAngleRate2],
                             0.);
  
  return true;
}

void TwoLinkManipulatorVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("state", "signal/vector", "Two-link manipulator state to visualize", state_));
}

void TwoLinkManipulatorVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
    throw Exception("visualization/tlm requires a configured visualizer to run");

  state_ = (VectorSignal*)config["state"].ptr();

  // Create window  
  create("Two-link manipulator");
}

void TwoLinkManipulatorVisualization::reconfigure(const Configuration &config)
{
}

void TwoLinkManipulatorVisualization::reshape(int width, int height)
{
  initProjection(-1, 1, -1, 1);
}

void TwoLinkManipulatorVisualization::idle()
{
  refresh();
}

void TwoLinkManipulatorVisualization::draw()
{
  clear();
  
  Vector state = state_->get();
  
  if (state.size())
  {
    double phi1 = state[0]+M_PI/2;
    double phi2 = state[1];
  
    drawLink(0, 0, 0.4*cos(phi1), 0.4*sin(phi1));
    drawLink(0.4*cos(phi1), 0.4*sin(phi1), 0.4*cos(phi1)+0.4*cos(phi1+phi2), 0.4*sin(phi1)+0.4*sin(phi1+phi2));
    drawJoint(0, 0);
    drawJoint(0.4*cos(phi1), 0.4*sin(phi1));
    drawMass(0.2*cos(phi1), 0.2*sin(phi1));
    drawMass(0.4*cos(phi1)+0.2*cos(phi1+phi2), 0.4*sin(phi1)+0.2*sin(phi1+phi2));
  }

  swap();
}
