/** \file leo2.cpp
 * \brief LEO/2 environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-07
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

#include <grl/environments/leo2.h>

using namespace grl;

REGISTER_CONFIGURABLE(LEO2Environment)

void LEO2Environment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("port", "Device ID of FTDI usb-to-serial converter", port_, CRP::Configuration));
  config->push_back(CRP("bps", "Bit rate", bps_, CRP::Configuration));
  config->push_back(CRP("state", "state", "Current state of the robot", CRP::Provided));
}

void LEO2Environment::configure(Configuration &config)
{
  port_ = config["port"].str();
  bps_ = config["bps"];
  
  state_obj_ = new VectorSignal();
  
  config.set("state", state_obj_);
  
  if (ftdi_.open(port_) < 0)
    WARNING("Error opening port " << port_ << ": " << ftdi_.error_string());
  
  if (ftdi_.set_baud_rate(bps_) < 0)
    WARNING("Error setting bit rate " << bps_ << ": " << ftdi_.error_string());
  
  NOTICE("Waiting for LEO/2 controller");

  readState();
  
  INFO("Communication with LEO/2 controller established");
}

void LEO2Environment::reconfigure(const Configuration &config)
{
}
    
void LEO2Environment::start(int test, Observation *obs)
{
  // First await robot to self-right
  do
  {
    readState();
    
    if (mode_ != modeRighting && mode_ != modeAwaitControl)
      selfRight();
  } while (mode_ != modeAwaitControl);
  
  // Then bring it into the starting position
  writeControls(VectorConstructor(0.));
  
  Vector state = readState();
  state_obj_->set(state);
  
  *obs = VectorConstructor(state[0], state[1]);
  obs->absorbing = false;

  timer_.restart();
}

double LEO2Environment::step(const Action &action, Observation *obs, double *reward, int *terminal)
{
  writeControls(action);

  Vector state = readState();
  state_obj_->set(state);
  
  *obs = VectorConstructor(state[0], state[1]);
  obs->absorbing = false;
  
  double tau = timer_.elapsed();
  timer_.restart();
  
  *reward = 0;
  *terminal = 0;
  
  return tau;
}

Vector LEO2Environment::readState()
{
  int n = 0;
  unsigned char buf[256];

  do
  {
    ftdi_.read(buf, 1);
  } while (*buf != 0xFF);

  do
  {
    n += ftdi_.read(buf+n, 5-n);
    if (n < 0)
    {
      ERROR("Could not read state:" << ftdi_.error_string());
      return Vector();
    }
  } while (n < 5);
  
  mode_ = buf[0];
  
  return VectorConstructor((buf[1]+(buf[2]<<7))*0.088*M_PI/180, ((buf[3]+(buf[4]<<7))&1023)*((buf[4]&4)?-1:1)*0.11/60*2*M_PI);
}

void LEO2Environment::writeControls(const Vector &u)
{
  unsigned char buf[256];
  
  buf[0] = 0xFF;
  buf[1] = 0;
  buf[2] = fmin(fabs(u[0]), 1)*127; if (u[0] < 0) buf[2] |= 128;
  
  ftdi_.write(buf, 3);
}

void LEO2Environment::selfRight()
{
  INFO("Starting self-righting procedure");
  
  unsigned char buf[256];
  
  buf[0] = 0xFF;
  buf[1] = 1;
  buf[2] = 0;
  
  ftdi_.write(buf, 3);
}
