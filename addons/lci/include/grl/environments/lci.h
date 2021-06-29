/** \file lci.h
 * \brief LCI environment definition.
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

#ifndef GRL_LCI_H_
#define GRL_LCI_H_

#include <grl/environment.h>
#include <termios.h>

namespace grl
{

class Serial
{
  protected:
    int fd_;
    
  public:
    Serial() : fd_(-1) { }
    
    int open(std::string portname, int bps=B115200);
    int read(unsigned char *buf, int sz);
    void write(const unsigned char *buf, int sz);
};

/// LCI Pendulum environment
class LCIPendulumEnvironment : public Environment
{
public:
  TYPEINFO("environment/lci_pendulum", "LCI Pendulum environment")

public:
  std::string port_;
  int bps_;
  Serial serial_;
  timer timer_;
  double time_, timeout_;
  Vector prev_state_;

public:
  LCIPendulumEnvironment() : port_("i:0x2341:0x0042"), bps_(115200), time_(0), timeout_(10) { }

  // From Configurable
  virtual void request(ConfigurationRequest* config);
  virtual void configure(Configuration& config);
  virtual void reconfigure(const Configuration& config);

  // From Environment
  virtual void start(int test, Observation* obs);
  virtual double step(const Action& action, Observation* obs, double* reward, int* terminal);

protected:
  virtual Vector readState();
  virtual void writeControls(const Vector& u);
};

/// LCI Cart-Pole environment
class LCICartPoleEnvironment : public Environment
{
  public:
    TYPEINFO("environment/lci_cartpole", "LCI Cart-Pole environment")

  public:
    std::string port_;
    int bps_;
    Serial serial_;
    timer timer_;
    double time_, timeout_;
    Vector prev_state_;

  public:
    LCICartPoleEnvironment() : port_("i:0x2341:0x0042"), bps_(115200), time_(0), timeout_(10) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Environment
    virtual void start(int test, Observation *obs);
    virtual double step(const Action &action, Observation *obs, double *reward, int *terminal);
    
  protected:
    virtual Vector readState();
    virtual void writeControls(const Vector &u);
};

}

#endif /* GRL_LCI_H_ */
