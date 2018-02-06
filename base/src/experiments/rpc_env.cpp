/** \file rpc_env.cpp
 * \brief RPC interface to environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-11-06
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

#include <stdio.h> 
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>  
#include <netinet/in.h>
#include <netdb.h>
#include <endian.h>

#include <grl/experiments/rpc_env.h>

using namespace grl;

REGISTER_CONFIGURABLE(RPCEnvExperiment)

void RPCEnvExperiment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("port", "Listen port", port_));
  config->push_back(CRP("environment", "environment", "Environment to interface", environment_));
}

void RPCEnvExperiment::configure(Configuration &config)
{
  port_ = config["port"];
  environment_ = (Environment*)config["environment"].ptr();
}

void RPCEnvExperiment::reconfigure(const Configuration &config)
{
  config.get("port", port_);
}

LargeVector RPCEnvExperiment::run()
{
  struct sockaddr_in myaddr, agentaddr;
  int yes = 1, fd;
  socklen_t addrlen = sizeof(agentaddr);

  grl_assert((fd = socket(AF_INET, SOCK_STREAM, 0)) >= 0);
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  bzero((char *) &myaddr, sizeof(myaddr));
  myaddr.sin_family = AF_INET;
  myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  myaddr.sin_port = htons(port_);
  
  if (bind(fd, (struct sockaddr *) &myaddr, sizeof(myaddr)) < 0)
  {
    perror("bind");
    throw Exception("Bind failed");
  }
  
  NOTICE("Waiting for client to connect");
  
  grl_assert(listen(fd, 1) >= 0);
  grl_assert((socket_ = accept(fd, (struct sockaddr *) &agentaddr, &addrlen)) >= 0);
  
  NOTICE("Client connected");

  while (1)
  {
    Observation obs;
    Action action = readVector();
    double reward = 0, tau = 0;
    int terminal = 0;
    
    if (action.size())
      tau = environment_->step(action, &obs, &reward, &terminal);
    else
      environment_->start(0, &obs);
      
    CRAWL(action << " - " << reward << " -> " << obs);
      
    writeVector(obs);
    writeDouble(reward);
    writeChar(terminal);
    writeDouble(tau);
  }

  return LargeVector();
}

void RPCEnvExperiment::writeChar(unsigned char c)
{
  grl_assert(write(socket_, &c, 1) == 1);
}

void RPCEnvExperiment::writeDouble(double d)
{
  grl_assert(write(socket_, (unsigned char*) &d, sizeof(double)) == sizeof(double));
}

void RPCEnvExperiment::writeVector(const Vector &v)
{
  writeChar(v.size());
  for (size_t ii=0; ii < v.size(); ++ii)
    writeDouble(v[ii]);
}

void RPCEnvExperiment::writeState(const Vector &state, double reward, bool terminal, bool absorbing)
{
  writeVector(state);
  writeDouble(reward);
  writeChar(absorbing?2:terminal?1:0);
}

Vector RPCEnvExperiment::readVector()
{
  unsigned char c;

  grl_assert(read(socket_, &c, 1) == 1);
  
  Vector v(c);
  
  for (size_t ii=0; ii < c; ++ii)
    grl_assert(read(socket_, (unsigned char*)&v[ii], sizeof(double)) == sizeof(double));

  return v;    
}
