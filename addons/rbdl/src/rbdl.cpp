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
}

void RBDLDynamics::configure(Configuration &config)
{
  file_ = config["file"].str();
  
  struct stat buffer;   
  if (stat (file_.c_str(), &buffer) != 0)
    file_ = std::string(RBDL_LUA_CONFIG_DIR) + "/" + file_;
  
  model_ = new RigidBodyDynamics::Model();

  INFO("Loading model from " << file_);
  if (!RigidBodyDynamics::Addons::LuaModelReadFromFile(file_.c_str(), model_))
  {
    ERROR("Error loading model " << file_);
    throw bad_param("dynamics/rbdl:file");
  }

  for (unsigned int i = 1; i < model_->mBodies.size(); i++)
  {
    using namespace RigidBodyDynamics;
    using namespace RigidBodyDynamics::Math;
    Body &body = model_->mBodies[i];
    SpatialRigidBodyInertia body_rbi = SpatialRigidBodyInertia::createFromMassComInertiaC(body.mMass, body.mCenterOfMass, body.mInertia);
    std::cout << "=============== Spatial inertia of body " << i << " ===============" << std::endl;
    std::cout << body_rbi.toMatrix() << std::endl << std::endl;
  }
  
  NOTICE("Loaded RBDL model with " << model_->dof_count << " degrees of freedom");
  
  L_ = luaL_newstate();
  luaL_openlibs(L_);
  
  // Add script path to search directory
  char buf[PATH_MAX];
  strcpy(buf, file_.c_str());
  std::string ls = "package.path = package.path .. ';";
  ls = ls + dirname(buf) + "/?.lua'";
  luaL_dostring(L_, ls.c_str()); 
  
  luaL_dofile(L_, file_.c_str());
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
  size_t dim = model_->dof_count;
  
  if (state.size() != 2*dim+1)
    throw Exception("dynamics/rbdl is incompatible with specified task");
    
  // Convert action to forces and torques
  Vector controls;
  lua_getglobal(L_, "control");  /* function to be called */
  if (!lua_isnil(L_, -1))
  {
    lua_pushvector(L_, state);
    lua_pushvector(L_, action);
    if (lua_pcall(L_, 2, 1, 0) != 0)
    {
      ERROR("Cannot find controls: " << lua_tostring(L_, -1));
      lua_pop(L_, 1);
      throw bad_param("dynamics/rbdl:file");
    }
    controls = lua_tovector(L_, -1);
    if (controls.size() != dim)
      throw Exception("Controller is incompatible with dynamics");
  }
  else
  {
    controls = action;
    if (controls.size() != dim)
      throw Exception("Policy is incompatible with dynamics");
  }

  lua_pop(L_, 1);

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

  RigidBodyDynamics::ForwardDynamics(*model_, q, qd, u, qdd);

  xd->resize(2*dim+1);

  for (size_t ii=0; ii < dim; ++ii)
  {
    (*xd)[ii] = qd[ii];
    (*xd)[ii + dim] = qdd[ii];
  }
  (*xd)[2*dim] = 1.;
}

void LuaTask::request(ConfigurationRequest *config)
{
  Task::request(config);
  
  config->push_back(CRP("file", "Lua task file", file_, CRP::Configuration));
  config->push_back(CRP("options", "Option string to pass to task configuration function", options_, CRP::Configuration));
}

void LuaTask::configure(Configuration &config)
{
  file_ = config["file"].str();
  options_ = config["options"].str();
  
  struct stat buffer;   
  if (stat (file_.c_str(), &buffer) != 0)
    file_ = std::string(RBDL_LUA_CONFIG_DIR) + "/" + file_;
  
  L_ = luaL_newstate();
  luaL_openlibs(L_);
  
  if (luaL_dofile(L_, file_.c_str()))
  {
    ERROR("Error loading task " << file_ << ": " << lua_tostring(L_, -1));
    throw bad_param("task/lua:file");
  }
  
  lua_getglobal(L_, "configure");
  lua_pushstring(L_, options_.c_str());
  if (lua_pcall(L_, 1, 1, 0) != 0)
  {
    ERROR("Cannot configure task: " << lua_tostring(L_, -1));
    throw bad_param("task/lua:file");
  }
  
  if (!lua_istable(L_, -1))
  {
    ERROR("Task configuration should return a table");
    throw bad_param("task/lua:file");
  }
  
  config.set("observation_dims", lua_gettablenumber(L_, "observation_dims"));
  config.set("observation_min", lua_gettablevector(L_, "observation_min"));
  config.set("observation_max", lua_gettablevector(L_, "observation_max"));
  config.set("action_dims", lua_gettablenumber(L_, "action_dims"));
  config.set("action_min", lua_gettablevector(L_, "action_min"));
  config.set("action_max", lua_gettablevector(L_, "action_max"));
  config.set("reward_min", lua_gettablenumber(L_, "reward_min"));
  config.set("reward_max", lua_gettablenumber(L_, "reward_max"));
  
  lua_pop(L_, 1);
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
  lua_getglobal(L_, "start");
  lua_pushnumber(L_, test);
  if (lua_pcall(L_, 1, 1, 0) != 0)
  {
    ERROR("Cannot find start state: " << lua_tostring(L_, -1));
    lua_pop(L_, 1);
    throw bad_param("task/lua:file");
  }
  
  *state = lua_tovector(L_, -1);
  lua_pop(L_, 1);
}
 
void LuaTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  lua_getglobal(L_, "observe");  /* function to be called */
  lua_pushvector(L_, state);
  if (lua_pcall(L_, 1, 2, 0) != 0)
  {
    ERROR("Cannot observe state: " << lua_tostring(L_, -1));
    lua_pop(L_, 2);
    throw bad_param("task/lua:file");
  }
  
  *obs = lua_tovector(L_, -2);
  
  if (!lua_isnumber(L_, -1))
    WARNING("Termination condition is not a number");
  
  *terminal = lua_tointeger(L_, -1);
  lua_pop(L_, 2);
}

void LuaTask::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  lua_getglobal(L_, "evaluate");  /* function to be called */
  lua_pushvector(L_, state);
  lua_pushvector(L_, action);
  lua_pushvector(L_, next);
  if (lua_pcall(L_, 3, 1, 0) != 0)
  {
    ERROR("Cannot evaluate reward: " << lua_tostring(L_, -1));
    lua_pop(L_, 1);
    throw bad_param("task/lua:file");
  }
  
  if (!lua_isnumber(L_, -1))
    WARNING("Reward is not a number");

  *reward = lua_tonumber(L_, -1);
  lua_pop(L_, 1);
}
 
bool LuaTask::invert(const Vector &obs, Vector *state) const
{
  lua_getglobal(L_, "invert");  /* function to be called */
  lua_pushvector(L_, obs);
  if (lua_pcall(L_, 1, 1, 0) != 0)
  {
    WARNING("Cannot invert observation: " << lua_tostring(L_, -1));
    lua_pop(L_, 1);
    return false;
  }
  
  *state = lua_tovector(L_, -1);
  lua_pop(L_, 1);
  
  return true;
}
