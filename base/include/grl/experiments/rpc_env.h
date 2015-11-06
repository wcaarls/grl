/** \file rpc_env.h
 * \brief RPC interface to environment header file.
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

#ifndef GRL_RPC_ENV_EXPERIMENT_H_
#define GRL_RPC_ENV_EXPERIMENT_H_

#include <grl/experiment.h>
#include <grl/environment.h>

namespace grl
{

/// RPC server exposing an environment
class RPCEnvExperiment : public Experiment
{
  public:
    TYPEINFO("experiment/rpc/environment", "Environment RPC server")

  protected:
    int port_;
    Environment *environment_;
    
    int socket_;
    
  public:
    RPCEnvExperiment() : port_(31033), environment_(NULL), socket_(-1) { }
    
    ~RPCEnvExperiment()
    {
      if (socket_ > -1)
        close(socket_);
    }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Experiment
    virtual RPCEnvExperiment *clone() const;
    virtual void run();

  protected:
    void writeChar(unsigned char c);
    void writeDouble(double d);
    void writeVector(const Vector &v);
    void writeState(const Vector &state, double reward, bool terminal, bool absorbing);
    Vector readVector();
};

}

#endif /* GRL_RPC_ENV_EXPERIMENT_H_ */
