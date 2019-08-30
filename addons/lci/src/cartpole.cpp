/** \file cartpole.cpp
 * \brief LCI Cart-Pole environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2019-08-30
 *
 * \copyright \verbatim
 * Copyright (c) 2019, Wouter Caarls
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

#include <grl/environments/lci.h>

using namespace grl;

REGISTER_CONFIGURABLE(LCICartPoleEnvironment)

void LCICartPoleEnvironment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("port", "Serial port of Arduino MEGA", port_, CRP::Configuration));
  config->push_back(CRP("bps", "Bit rate", bps_, CRP::Configuration));
  
  config->push_back(CRP("timeout", "Timeout", timeout_, CRP::Configuration, 0., 3600.));
}

void LCICartPoleEnvironment::configure(Configuration &config)
{
  port_ = config["port"].str();
  bps_ = config["bps"];
  timeout_ = config["timeout"];

  if (serial_.open(port_, B115200) < 0)
    WARNING("Error opening port " << port_);
  
  NOTICE("Waiting for controller");

  readState();
  
  INFO("Communication with controller established");
}

void LCICartPoleEnvironment::reconfigure(const Configuration &config)
{
}
    
void LCICartPoleEnvironment::start(int test, Observation *obs)
{
  // Move to zero position
  
  NOTICE("Moving to zero position");
  
  Vector state, action(1);
  do
  {
    state  = readState();
    action[0] = -0.3 + 0.6*(state[0] > 0);
    writeControls(action);
  } while (fabs(state[0]) > 0.01);

  action[0] = 0;
  writeControls(action);
  
  NOTICE("Waiting for pole zero position");
  
  do
  {
    state  = readState();
  } while (fabs(state[1]) > 0.01);
  
  *obs = VectorConstructor(state[0], state[1], 0., 0.);
  obs->absorbing = false;
  
  NOTICE("Starting episode");

  timer_.restart();
  time_ = 0;
  prev_state_ = state;
}

double LCICartPoleEnvironment::step(const Action &action, Observation *obs, double *reward, int *terminal)
{
  writeControls(action);

  Vector state = readState();
  double tau = timer_.elapsed();
  
  if (tau > 0.015)
    WARNING("Failed deadline");
    
  if (tau < 0.005)
  {
    WARNING("Buffered input");
    tau = 0.005;
  }
  
  timer_.restart();
  
  *obs = VectorConstructor(state[0], state[1], (state[0]-prev_state_[0])/tau, (state[1]-prev_state_[1])/tau);
  obs->absorbing = false;
  
  time_ += tau;
  
  double a = fmod(fabs(state[1]), 2*M_PI);
  if (a > M_PI) a -= 2*M_PI;
  
  *reward = -2*pow((*obs)[0], 2) -0.1*pow((*obs)[2], 2) -pow(a, 2) -0.1*pow((*obs)[3], 2);
  
  if (fabs(state[0]) > 0.2 || fabs((*obs)[2]) > 2)
    *terminal = 2;
  else if (time_ > timeout_)
    *terminal = 1;
  else
    *terminal = 0;
    
  if (*terminal == 2)
  {
    obs->absorbing = true;
    writeControls(VectorConstructor(0.));
  }
  
  prev_state_ = state;
  
  return tau;
}

Vector LCICartPoleEnvironment::readState()
{
  unsigned char buf[16];
  int len;

  do
  {
    do
    {
      len = serial_.read(buf, 1);
    } while (!len || buf[0] != 0xFF);
    len = serial_.read(buf, 8);
  } while (len != 8);
  
  Vector state = VectorConstructor(*(float*)&buf[0], *(float*)&buf[4]);
  
  return state;
}

void LCICartPoleEnvironment::writeControls(const Vector &u)
{
  unsigned char buf[5];
  
  buf[0] = 0xFF;
  *(float*)&buf[1] = u[0];
  
  serial_.write(buf, 5);
}
