/** \file leotask.h
 * \brief RBDL file for C++ description of Leo task.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2016-06-30
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
 
#ifndef GRL_RBDL_LEO_TASK_H_
#define GRL_RBDL_LEO_TASK_H_

#include <functional>
#include <grl/environment.h>
#include <grl/environments/leohelper.h>
#include <grl/environments/rbdl.h>

//namespace RigidBodyDynamics {}

namespace grl
{
class LeoRBDLDynamics : public RBDLDynamics // put here all stuff needed, if needed!
{
  public:
    TYPEINFO("dynamics/rbdl_leo", "RBDL rigid body dynamics of Leo")

  public:


  public:
    LeoRBDLDynamics() { }

    // From Configurable
    //virtual void request(ConfigurationRequest *config);
    //virtual void configure(Configuration &config);
    //virtual void reconfigure(const Configuration &config);

    // From Dynamics
//    virtual void finalize(Vector &state);

//  protected:
//    ModelHelpers::LeoHelper leo_helper_;
};

class LeoSquatTask : public Task
{
  public:
    TYPEINFO("task/leoSquat", "Task specification for Leo squatting")

  public:


  public:
    LeoSquatTask() : timeout_(10) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual LeoSquatTask *clone() const;
    virtual void start(int test, Vector *state) const;
    virtual void observe(const Vector &state, Vector *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const;
    virtual bool invert(const Vector &obs, Vector *state) const;
    virtual Matrix rewardHessian(const Vector &state, const Vector &action) const;

    int failed(const Vector &state) const;

  protected:

    int action_dims_;
    double timeout_;
};


}

#endif /* GRL_RBDL_LEO_TASK_H_ */
