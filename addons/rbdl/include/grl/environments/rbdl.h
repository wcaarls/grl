/** \file rbdl.h
 * \brief RBDL environment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-04-01
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
 
#ifndef GRL_RBDL_ENVIRONMENT_H_
#define GRL_RBDL_ENVIRONMENT_H_

#include <functional>
#include <grl/environment.h>
#include <rbdl/rbdl.h>
#include <grl/environments/LuaBasic.h>
#include <lua.h>

namespace RigidBodyDynamics {}

namespace grl
{

struct RBDLState
{
  class RigidBodyDynamics::Model *model;
  class lua_State *L;
  
  ~RBDLState()
  {
    delete model;
    lua_close(L);
  }
};

/// RBDL dynamics
class RBDLDynamics : public Dynamics
{
  public:
    TYPEINFO("dynamics/rbdl", "RBDL rigid body dynamics")

  public:
    std::string file_, options_;
    mutable Instance<RBDLState> rbdl_state_;
  
    mutable std::map<std::string, Point> points;
    mutable std::map<std::string, RigidBodyDynamics::ConstraintSet> constraints;

  public:
    RBDLDynamics() : rbdl_state_(std::bind(&RBDLDynamics::createRBDLState, this)) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Dynamics
    virtual void eom(const Vector &state, const Vector &actuation, Vector *xd) const;
    
    /// Adds positions of additional points to state vector.
    virtual void finalize(const Vector &state, Vector &out) const;
    
  protected:
    // own
    std::vector<std::string> points_, auxiliary_;

  protected:
    RBDLState *createRBDLState() const;
    bool loadPointsFromFile(const char* filename, RigidBodyDynamics::Model *model) const;
    bool loadConstraintSetsFromFile(const char* filename, RigidBodyDynamics::Model *model) const;
    void getPointPosition(const Vector &state, const std::string point_name, Vector &out) const;
    void getAuxiliary(const Vector &state, double &modelMass, Vector &centerOfMass, Vector &centerOfMassVelocity, Vector &angularMomentum) const;
};

struct LuaState
{
  class lua_State *L;
  
  ~LuaState()
  {
    lua_close(L);
  }
};

class LuaTask : public Task
{
  public:
    TYPEINFO("task/lua", "User-provided task specification in LUA")
    
  public:
    std::string file_, options_;
    mutable Instance<LuaState> lua_state_;
  
  public:
    LuaTask() : lua_state_(std::bind(&LuaTask::createLuaState, this)) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void start(int test, Vector *state) const;
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const;
    virtual bool invert(const Observation &obs, Vector *state) const;
    virtual Matrix rewardHessian(const Observation &state, const Action &action) const;
    
  protected:
    LuaState *createLuaState() const;
};

}

#endif /* GRL_RBDL_ENVIRONMENT_H_ */
