/** \file leo2.h
 * \brief LEO/2 environment definition.
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

#ifndef GRL_LEO2_H_
#define GRL_LEO2_H_

#include <ftdi.hpp>
#include <grl/environment.h>

namespace grl
{

/// LEO/2 environment
class LEO2Environment : public Environment
{
  public:
    TYPEINFO("environment/leo2", "LEO/2 environment")
    enum mode {modeIdle, modeRighting, modeAwaitControl, modeParking, modeControl};

  public:
    std::string port_;
    int bps_;
  
    Ftdi::Context ftdi_;
    timer timer_;
    int mode_;
    State *state_obj_;

  public:
    LEO2Environment() : port_("i:0x0403:0x6001"), bps_(57600), mode_(0), state_obj_(NULL) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Environment
    virtual void start(int test, Vector *obs);
    virtual double step(const Vector &action, Vector *obs, double *reward, int *terminal);
    
  protected:
    virtual Vector readState();
    virtual void writeControls(const Vector &u);
    virtual void selfRight();
};

}

#endif /* GRL_LEO2_H_ */
