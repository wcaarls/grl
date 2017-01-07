/** \file lua_deployer.cpp
 * \brief Deployer for object configurations specified through Lua.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-12-28
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Wouter Caarls
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

#include <libgen.h>

#include <grl/grl.h>
#include <grl/configurable.h>
#include <grl/experiment.h>
#include <grl/lua_utils.h>

using namespace grl;
using namespace std;

static int yaml(lua_State *L);
static int instantiate(lua_State *L);
static int run(lua_State *L);
static int reload(lua_State *L);

Configurator *lua_toconfigurator(lua_State *L, int index)
{
  luaL_checktype(L, index, LUA_TTABLE);
  lua_pushstring(L, "ptr");
  lua_rawget(L, index);
  
  if (!lua_islightuserdata(L, -1))
    luaL_error(L, "Argument #%d is not an object", index);
    
  Configurator *conf = (Configurator*)lua_touserdata(L, -1);
  
  if (!conf)
    luaL_error(L, "Argument #%d points to an invalid object", index);
    
  lua_pop(L, 1); // conf
  
  return conf;
}

void lua_newgrl(lua_State *L, Configurator *conf)
{
  lua_newtable(L);
  luaL_getmetatable(L, "grl.object");
  lua_setmetatable(L, -2);
  lua_pushstring(L, "ptr");
  lua_pushlightuserdata(L, conf);
  lua_rawset(L, -3);
  lua_pushstring(L, "yaml");
  lua_pushcfunction(L, yaml);
  lua_rawset(L, -3);
  lua_pushstring(L, "instantiate");
  lua_pushcfunction(L, instantiate);
  lua_rawset(L, -3);
  lua_pushstring(L, "run");
  lua_pushcfunction(L, run);
  lua_rawset(L, -3);
  lua_pushstring(L, "load");
  lua_pushcfunction(L, reload);
  lua_rawset(L, -3);
}

static int _new(lua_State *L)
{
  Configurator *conf;
  if (lua_gettop(L))
  {
    std::string type = luaL_checkstring(L, 1);
    conf = new ObjectConfigurator("", type);
  }
  else
    conf = new Configurator();
    
  lua_newgrl(L, conf);
  
  return 1;
}

static int ref(lua_State *L)
{
  if (lua_gettop(L) != 1)
    luaL_error(L, "ref() expected 1 argument, %d given", lua_gettop(L));
  
  std::string path = luaL_checkstring(L, 1);
  Configurator *conf = new ParameterConfigurator("", path);
  lua_newgrl(L, conf);
  
  return 1;
}

static int load(lua_State *L)
{
  if (lua_gettop(L) != 1)
    luaL_error(L, "load() expected 1 argument, %d given", lua_gettop(L));
  
  std::string file = luaL_checkstring(L, 1);
  
  INFO("Loading configuration from '" << file << "'");
  
  Configurator *conf = loadYAML(file);
  if (!conf)
    luaL_error(L, "Could not load configuration from %s", file.c_str());

  lua_newgrl(L, conf);
  
  return 1;
}

static int reload(lua_State *L)
{
  if (lua_gettop(L) != 2)
    luaL_error(L, "load() expected 2 arguments, %d given", lua_gettop(L));
  
  Configurator *conf = lua_toconfigurator(L, 1);
  std::string file = luaL_checkstring(L, 2);
  
  INFO("Loading configuration from '" << file << "'");
  
  conf = loadYAML(file, "", conf);
  if (!conf)
    luaL_error(L, "Could not load configuration from %s", file.c_str());
    
  lua_pushvalue(L, 1);
  
  return 1;
}

static int index(lua_State *L)
{
  if (lua_gettop(L) != 2)
    luaL_error(L, "__index() expected 2 arguments, %d given", lua_gettop(L));
  
  Configurator *conf = lua_toconfigurator(L, 1);
  if (dynamic_cast<ParameterConfigurator*>(conf))
    luaL_error(L, "Cannot index a parameter");
  
  std::string key = luaL_checkstring(L, 2);
  
  Configurator *param = conf->find(key);
  
  // If parameter doesn't exist, assume it will be defined upon
  // instantiation (i.e. uses default value, or it's provided)
  if (!param)
    param = new ReferenceConfigurator("", conf, key);
  
  lua_newgrl(L, param);

  return 1;
}

static int newindex(lua_State *L)
{
  if (lua_gettop(L) != 3)
    luaL_error(L, "__newindex() expected 3 arguments, %d given", lua_gettop(L));
    
  Configurator *conf = lua_toconfigurator(L, 1);
  if (dynamic_cast<ParameterConfigurator*>(conf))
    luaL_error(L, "Cannot index a parameter");
  if (dynamic_cast<ReferenceConfigurator*>(conf))
    luaL_error(L, "Cannot assign to a reference");

  std::string key = luaL_checkstring(L, 2);

  if (lua_istable(L, 3))
  {
    // Vector or configurator
    lua_pushstring(L, "ptr");
    lua_gettable(L, 3);
    if (lua_islightuserdata(L, -1))
    {
      // Configurator
      Configurator *conf2 = (Configurator*)lua_touserdata(L, -1);
      
      if (!conf2)
        luaL_error(L, "Argument #3 points to an invalid object");
      
      if (conf2->parent())
        new ReferenceConfigurator(key, conf2, "", conf);
      else
        conf2->graft(key, conf);
    }
    else
    {
      // Vector
      LargeVector v = lua_tovector(L, -2);
      std::ostringstream oss;
      std::vector<double> v_out;
      fromVector(v, v_out);
      oss << v_out;
      new ParameterConfigurator(key, oss.str(), conf);
    }
    lua_pop(L, 1);
  }
  else
  {
    std::ostringstream oss;
    
    // Regular value
    if (lua_isnumber(L, 3))
      oss << lua_tonumber(L, 3);
    else if (lua_isstring(L, 3))
      oss << lua_tostring(L, 3);
      
    if (!oss.str().empty())
      new ParameterConfigurator(key, oss.str(), conf);
  }

  return 0;
}

static int yaml(lua_State *L)
{
  if (lua_gettop(L) != 1)
    luaL_error(L, "yaml() expected 1 argument, %d given", lua_gettop(L));
    
  Configurator *conf = lua_toconfigurator(L, 1);

  lua_pushstring(L, conf->yaml().c_str());    

  return 1;
}

static int instantiate(lua_State *L)
{
  if (lua_gettop(L) != 1)
    luaL_error(L, "instantiate() expected 1 argument, %d given", lua_gettop(L));
    
  Configurator *conf = lua_toconfigurator(L, 1);
  
  lua_newgrl(L, conf->instantiate());

  return 1;
}

static int tostring(lua_State *L)
{
  if (lua_gettop(L) != 1)
    luaL_error(L, "__tostring() expected 1 argument, %d given", lua_gettop(L));
    
  Configurator *conf = lua_toconfigurator(L, 1);
  lua_pushstring(L, conf->str().c_str());
  
  return 1;
}

static int call(lua_State *L)
{
  if (lua_gettop(L) != 2)
    luaL_error(L, "__call() expected 2 arguments, %d given", lua_gettop(L));

  lua_toconfigurator(L, 1);
  luaL_checktype(L, 2, LUA_TTABLE);
    
  // Start configuration table iteration
  lua_getmetatable(L, 1);
  lua_pushvalue(L, 2);
  lua_pushnil(L);
  while (lua_next(L, -2))
  {
    lua_pushstring(L, "__newindex");
    lua_gettable(L, -5);  //__newindex
    lua_pushvalue(L, 1);  //configurator
    lua_pushvalue(L, -4); //key
    lua_pushvalue(L, -4); //value
    lua_call(L, 3, 0);

    lua_pop(L, 1); // Pop value
  }

  // Pop key, table and metatable
  lua_pop(L, 3);
  
  return 1;
}

static int run(lua_State *L)
{
  if (lua_gettop(L) != 1)
    luaL_error(L, "run() expected 1 argument, %d given", lua_gettop(L));
    
  Configurator *conf = lua_toconfigurator(L, 1);
  
  if (!conf->ptr())
    luaL_error(L, "run() requires an instantiated object");
    
  Experiment *experiment = dynamic_cast<Experiment*>(conf->ptr());
  if (!experiment)
    luaL_error(L, "run() requires an object of type experiment, got %s", conf->ptr()->d_type().c_str());
    
  configurator_ = conf->root();
  experiment->run();
  
  return 0;
}

static const struct luaL_reg grllib [] = {
  {"new", _new},
  {"ref", ref},
  {"load", load},
  {NULL, NULL}
};

int main(int argc, char **argv)
{
  iSIGSEGV();

  int seed = 0;

  int c;
  while ((c = getopt (argc, argv, "vs:c")) != -1)
  {
    switch (c)
    {
      case 'v':
        grl_log_verbosity__++;
        break;
      case 's':
        seed = atoi(optarg);
        break;
      default:
        return 1;    
    }
  }

  if (optind > argc-1)
  {
    ERROR("Usage: " << endl << "  " << argv[0] << " [-v] [-s seed] <Lua file> [Lua file...]");
    return 1;
  }
  
  if (seed)
  {
    srand(seed);
    srand48(seed);
  }
  else
  {
    srand(time(NULL));
    srand48(time(NULL));
  }
  
  // Load plugins
  loadPlugins();

  iSIGINT();
  
  // Lua initialization
  lua_State *L = luaL_newstate();
  luaL_openlibs(L);
  luaL_openlib(L, "grl", grllib, 0);
  luaL_newmetatable(L, "grl.object");
  lua_pushstring(L, "__index");
  lua_pushcfunction(L, index);
  lua_settable(L, -3);
  lua_pushstring(L, "__newindex");
  lua_pushcfunction(L, newindex);
  lua_settable(L, -3);
  lua_pushstring(L, "__tostring");
  lua_pushcfunction(L, tostring);
  lua_settable(L, -3);
  lua_pushstring(L, "__call");
  lua_pushcfunction(L, call);
  lua_settable(L, -3);
  
  for (; optind < argc; ++optind)
  {
    NOTICE("Loading configuration from '" << argv[optind] << "'");
    
    char buf[PATH_MAX];
    strcpy(buf, argv[optind]);
    std::string ls = "package.path = package.path .. ';";
    ls = ls + dirname(buf) + "/?.lua'";
    luaL_dostring(L, ls.c_str()); 
    if (luaL_dofile(L, argv[optind]))
      ERROR("Error loading configuration: " << lua_tostring(L, -1));
  }
  
  NOTICE("Exiting");

  return 0;
}
