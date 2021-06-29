/** \file pendulum.cpp
 * \brief LCI Pendulum environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2021-06-24
 *
 * \copyright \verbatim
 * Copyright (c) 2021, Wouter Caarls
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
#include <unistd.h>

using namespace grl;

REGISTER_CONFIGURABLE(LCIPendulumEnvironment)

void LCIPendulumEnvironment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("port", "Serial port of Arduino MEGA", port_, CRP::Configuration));
  config->push_back(CRP("bps", "Bit rate", bps_, CRP::Configuration));
  
  config->push_back(CRP("timeout", "Timeout", timeout_, CRP::Configuration, 0., 3600.));
}

void LCIPendulumEnvironment::configure(Configuration &config)
{
  port_ = config["port"].str();
  bps_ = config["bps"];
  timeout_ = config["timeout"];

  while (serial_.open(port_, B115200) < 0)
  {
    WARNING("Error opening port " << port_);
    usleep(1000000);
  }

  config.set("observation_dims", 2);
  config.set("observation_min", VectorConstructor(0., -12 * M_PI));
  config.set("observation_max", VectorConstructor(2 * M_PI, 12 * M_PI));
  config.set("action_dims", 1);
  config.set("action_min", VectorConstructor(-1));
  config.set("action_max", VectorConstructor(1));
  config.set("reward_min", -5 * pow(M_PI, 2) - 0.1 * pow(12 * M_PI, 2) - 1 * pow(1, 2));
  config.set("reward_max", 0);
  
  NOTICE("Waiting for controller");

  readState();
  
  INFO("Communication with controller established");
}

void LCIPendulumEnvironment::reconfigure(const Configuration &config)
{
}
    
void LCIPendulumEnvironment::start(int test, Observation *obs)
{
  NOTICE("Moving to down position");
  
  Vector state, action(1);
  do
  {
    // TODO: deal with wrapping
    state = readState();
    action[0] = 0.6*state[0];
    writeControls(action);
  } while (fabs(state[0]) > 0.01);

  action[0] = 0;
  writeControls(action);
  
  NOTICE("Starting episode");

  timer_.restart();
  time_ = 0;
}

double LCIPendulumEnvironment::step(const Action &action, Observation *obs, double *reward, int *terminal)
{
  writeControls(action);

  Vector state = readState();
  double tau = timer_.elapsed();
  timer_.restart();
  
//  NOTICE("tau: " << tau);

  if (tau > 0.1)
    WARNING("Failed deadline (" << tau << ")");
    
  if (tau < 0.025)
  {
    WARNING("Buffered input (" << tau << ")");
    tau = 0.005;
  }

  // Convert to [0, 2pi] with pi at top
  state[0] = fmod(fabs(state[0] + 2*M_PI), 2 * M_PI);
  
  *obs = state;
  obs->absorbing = false;
  
  time_ += tau;
  
  *reward = -5*pow(state[0] - M_PI, 2) -0.1*pow(state[1], 2) -pow(action[0], 2);
  
  if (time_ > timeout_)
    *terminal = 1;
  else
    *terminal = 0;
    
  return tau;
}

#define be32tohl(x) ((int64_t)be32toh(x))

// [-pi, pi] with zero at bottom
Vector LCIPendulumEnvironment::readState()
{
  unsigned char buf[8];
  int len;

  do
  {
    do
    {
      serial_.read(buf, 1);
    } while (buf[0] != 0xff);
    len = serial_.read(buf, 8);
  } while (len != 8);

//  for (int ii = 0; ii != len; ++ii)  
//    NOTICE("buf[" << ii << "]: " << (int)buf[ii]);
    
//  long int ang = be32tohl(*((uint32_t*)&buf[0])),
//           vel = be32tohl(*((uint32_t*)&buf[4]));
  long int ang = *((int32_t*)&buf[0]),
           vel = *((int32_t*)&buf[4]);
           
//  NOTICE("ang: " << ang << ", vel: " << vel);

  // 1000ppr encoder, quadrature decoding
  Vector state = VectorConstructor(2*M_PI*ang/4000.,
                                   2*M_PI*vel/4000.);
                                   
//  NOTICE("State: " << state);
  
  return state;
}

void LCIPendulumEnvironment::writeControls(const Vector &u)
{
  char pwm = (char) (u[0] * 127);
  
  serial_.write((unsigned char*)&pwm, 1);
}
