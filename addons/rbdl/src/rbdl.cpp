/** \file rbdl.cpp
 * \brief RBDL environment source file.
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

#include <sys/stat.h>
#include <libgen.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>

#include <grl/lua_utils.h>
#include <grl/environments/rbdl.h>

using namespace grl;

REGISTER_CONFIGURABLE(RBDLDynamics)
REGISTER_CONFIGURABLE(LuaTask)

void RBDLDynamics::request(ConfigurationRequest *config)
{
  config->push_back(CRP("file", "RBDL Lua model file", file_, CRP::Configuration));
  config->push_back(CRP("options", "Lua string to execute when loading model", options_, CRP::Configuration));
}

void RBDLDynamics::configure(Configuration &config)
{
  file_ = config["file"].str();
  options_ = config["options"].str();
  
  struct stat buffer;   
  if (stat (file_.c_str(), &buffer) != 0)
    file_ = std::string(RBDL_LUA_CONFIG_DIR) + "/" + file_;
}

void RBDLDynamics::reconfigure(const Configuration &config)
{
}

RBDLDynamics *RBDLDynamics::clone() const
{
  return NULL;
}

void RBDLDynamics::eom(const Vector &state, const Vector &action, Vector *xd) const
{
  RBDLState *rbdl = rbdl_state_.instance();

  size_t dim = rbdl->model->dof_count;
  
  if (state.size() != 2*dim+1)
    throw Exception("dynamics/rbdl is incompatible with specified task");
    
  // Convert action to forces and torques
  Vector controls;
  lua_getglobal(rbdl->L, "control");  /* function to be called */
  if (!lua_isnil(rbdl->L, -1))
  {
    lua_pushvector(rbdl->L, state);
    lua_pushvector(rbdl->L, action);
    if (lua_pcall(rbdl->L, 2, 1, 0) != 0)
    {
      ERROR("Cannot find controls: " << lua_tostring(rbdl->L, -1));
      lua_pop(rbdl->L, 1);
      throw bad_param("dynamics/rbdl:file");
    }
    controls = lua_tovector(rbdl->L, -1);
    if (controls.size() != dim)
      throw Exception("Controller is incompatible with dynamics");
  }
  else
  {
    controls = action;
    if (controls.size() != dim)
      throw Exception("Policy is incompatible with dynamics");
  }

  lua_pop(rbdl->L, 1);

  RigidBodyDynamics::Math::VectorNd u = RigidBodyDynamics::Math::VectorNd::Zero(dim);
  RigidBodyDynamics::Math::VectorNd q = RigidBodyDynamics::Math::VectorNd::Zero(dim);
  RigidBodyDynamics::Math::VectorNd qd = RigidBodyDynamics::Math::VectorNd::Zero(dim);
  RigidBodyDynamics::Math::VectorNd qdd = RigidBodyDynamics::Math::VectorNd::Zero(dim);

  for (size_t ii=0; ii < dim; ++ii)
  {
    u[ii] = controls[ii];
    q[ii] = state[ii];
    qd[ii] = state[ii + dim];
  }

  RigidBodyDynamics::ForwardDynamics(*rbdl->model, q, qd, u, qdd);

  xd->resize(2*dim+1);

  for (size_t ii=0; ii < dim; ++ii)
  {
    (*xd)[ii] = qd[ii];
    (*xd)[ii + dim] = qdd[ii];
  }
  (*xd)[2*dim] = 1.;
}

RBDLState *RBDLDynamics::createRBDLState() const
{
  RBDLState *rbdl = new RBDLState;

  rbdl->L = luaL_newstate();
  luaL_openlibs(rbdl->L);
  
  // Add script path to search directory
  char buf[PATH_MAX];
  strcpy(buf, file_.c_str());
  std::string ls = "package.path = package.path .. ';";
  ls = ls + dirname(buf) + "/?.lua'";
  luaL_dostring(rbdl->L, ls.c_str()); 

  if (!options_.empty())
    luaL_dostring(rbdl->L, options_.c_str()); 

  INFO("Loading model from " << file_);
  
  if (luaL_dofile(rbdl->L, file_.c_str()))
  {
    ERROR("Error executing model file: " << lua_tostring(rbdl->L, -1));
    lua_pop(rbdl->L, 1);
    throw bad_param("dynamics/rbdl:file");
  }

  rbdl->model = new RigidBodyDynamics::Model();

  if (!RigidBodyDynamics::Addons::LuaModelReadFromLuaState(rbdl->L, rbdl->model))
  {
    ERROR("Error loading model " << file_);
    throw bad_param("dynamics/rbdl:file");
  }

  for (unsigned int i = 1; i < rbdl->model->mBodies.size(); i++)
  {
    using namespace RigidBodyDynamics;
    using namespace RigidBodyDynamics::Math;
    Body &body = rbdl->model->mBodies[i];
    SpatialRigidBodyInertia body_rbi = SpatialRigidBodyInertia::createFromMassComInertiaC(body.mMass, body.mCenterOfMass, body.mInertia);
    TRACE("=============== Spatial inertia of body " << i << " ===============");
    TRACE(body_rbi.toMatrix() << std::endl);
  }
  
  NOTICE("Loaded RBDL model with " << rbdl->model->dof_count << " degrees of freedom");
  
  return rbdl;
}

void LuaTask::request(ConfigurationRequest *config)
{
  Task::request(config);
  
  config->push_back(CRP("file", "Lua task file", file_, CRP::Configuration));
  config->push_back(CRP("options", "Lua string to execute when loading task", options_, CRP::Configuration));
}

void LuaTask::configure(Configuration &config)
{
  file_ = config["file"].str();
  options_ = config["options"].str();
  
  struct stat buffer;   
  if (stat (file_.c_str(), &buffer) != 0)
    file_ = std::string(RBDL_LUA_CONFIG_DIR) + "/" + file_;

  LuaState *lua = lua_state_.instance();
  
  lua_getglobal(lua->L, "configure");
  if (lua_pcall(lua->L, 0, 1, 0) != 0)
  {
    ERROR("Cannot configure task: " << lua_tostring(lua->L, -1));
    throw bad_param("task/lua:file");
  }
  
  if (!lua_istable(lua->L, -1))
  {
    ERROR("Task configuration should return a table");
    throw bad_param("task/lua:file");
  }
  
  config.set("observation_dims", lua_gettablenumber(lua->L, "observation_dims"));
  config.set("observation_min", lua_gettablevector(lua->L, "observation_min"));
  config.set("observation_max", lua_gettablevector(lua->L, "observation_max"));
  config.set("action_dims", lua_gettablenumber(lua->L, "action_dims"));
  config.set("action_min", lua_gettablevector(lua->L, "action_min"));
  config.set("action_max", lua_gettablevector(lua->L, "action_max"));
  config.set("reward_min", lua_gettablenumber(lua->L, "reward_min"));
  config.set("reward_max", lua_gettablenumber(lua->L, "reward_max"));

  lua_pop(lua->L, 1);
}

void LuaTask::reconfigure(const Configuration &config)
{
}
 
LuaTask *LuaTask::clone() const
{
  return new LuaTask(*this);
}
 
void LuaTask::start(int test, Vector *state) const
{
  LuaState *lua = lua_state_.instance();

  lua_getglobal(lua->L, "start");
  lua_pushnumber(lua->L, test);
  if (lua_pcall(lua->L, 1, 1, 0) != 0)
  {
    ERROR("Cannot find start state: " << lua_tostring(lua->L, -1));
    lua_pop(lua->L, 1);
    throw bad_param("task/lua:file");
  }
  
  *state = lua_tovector(lua->L, -1);
  lua_pop(lua->L, 1);
}
 
void LuaTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  LuaState *lua = lua_state_.instance();

  lua_getglobal(lua->L, "observe");  /* function to be called */
  lua_pushvector(lua->L, state);
  if (lua_pcall(lua->L, 1, 2, 0) != 0)
  {
    ERROR("Cannot observe state: " << lua_tostring(lua->L, -1));
    lua_pop(lua->L, 2);
    throw bad_param("task/lua:file");
  }
  
  *obs = lua_tovector(lua->L, -2);
  
  if (!lua_isnumber(lua->L, -1))
    WARNING("Termination condition is not a number");
  
  *terminal = lua_tointeger(lua->L, -1);
  lua_pop(lua->L, 2);
}

void LuaTask::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  LuaState *lua = lua_state_.instance();

  lua_getglobal(lua->L, "evaluate");  /* function to be called */
  lua_pushvector(lua->L, state);
  lua_pushvector(lua->L, action);
  lua_pushvector(lua->L, next);
  if (lua_pcall(lua->L, 3, 1, 0) != 0)
  {
    ERROR("Cannot evaluate reward: " << lua_tostring(lua->L, -1));
    lua_pop(lua->L, 1);
    throw bad_param("task/lua:file");
  }
  
  if (!lua_isnumber(lua->L, -1))
    WARNING("Reward is not a number");

  *reward = lua_tonumber(lua->L, -1);
  lua_pop(lua->L, 1);
}
 
bool LuaTask::invert(const Vector &obs, Vector *state) const
{
  LuaState *lua = lua_state_.instance();

  lua_getglobal(lua->L, "invert");  /* function to be called */
  lua_pushvector(lua->L, obs);
  if (lua_pcall(lua->L, 1, 1, 0) != 0)
  {
    WARNING("Cannot invert observation: " << lua_tostring(lua->L, -1));
    lua_pop(lua->L, 1);
    return false;
  }
  
  *state = lua_tovector(lua->L, -1);
  lua_pop(lua->L, 1);
  
  return true;
}

Matrix LuaTask::rewardHessian(const Vector &state, const Vector &action) const
{
  LuaState *lua = lua_state_.instance();
  Matrix hessian;

  lua_getglobal(lua->L, "rewardHessian");  /* function to be called */
  if (!lua_isnil(lua->L, -1))
  {
    lua_pushvector(lua->L, state);
    lua_pushvector(lua->L, action);
    if (lua_pcall(lua->L, 2, 1, 0) != 0)
    {
      WARNING("Cannot determine reward hessian: " << lua_tostring(lua->L, -1));
      lua_pop(lua->L, 1);
      return Matrix();
    }
  
    hessian = lua_tomatrix(lua->L, -1);
  }
  
  lua_pop(lua->L, 1);
  return hessian;
}

LuaState *LuaTask::createLuaState() const
{
  LuaState *lua = new LuaState;
  
  lua->L = luaL_newstate();
  luaL_openlibs(lua->L);
  
  // Add script path to search directory
  char buf[PATH_MAX];
  strcpy(buf, file_.c_str());
  std::string ls = "package.path = package.path .. ';";
  ls = ls + dirname(buf) + "/?.lua'";
  luaL_dostring(lua->L, ls.c_str()); 

  if (!options_.empty())
    luaL_dostring(lua->L, options_.c_str()); 

  if (luaL_dofile(lua->L, file_.c_str()))
  {
    ERROR("Error loading task " << file_ << ": " << lua_tostring(lua->L, -1));
    throw bad_param("task/lua:file");
  }
  
  return lua;
}
