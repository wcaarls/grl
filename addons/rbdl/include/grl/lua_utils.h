/** \file lua_utils.h
 * \brief Lua utility functions 
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-04-02
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

#ifndef GRL_LUA_UTILS_H_
#define GRL_LUA_UTILS_H_

extern "C"
{
  #include <lua.h>
  #include <lauxlib.h>
  #include <lualib.h>
}
         
#include <grl/configurable.h>

namespace grl {

inline void lua_pushvector(lua_State *L, const Vector &v)
{
  lua_newtable(L);
  for (size_t ii=0; ii < v.size(); ++ii)
  {
    lua_pushnumber(L, v[ii]);
    lua_rawseti(L, -2, ii);
  }
}

inline Vector lua_tovector(lua_State *L, int index)
{
  if (!lua_istable(L, index))
  {
    WARNING("Lua object at stack index " << index << " is not a table");
    return Vector();
  }

  Vector v(lua_objlen(L, index), 0.);

  for (size_t ii=0; ii < v.size(); ++ii)
  {
    lua_rawgeti(L, index, ii+1);
    if (lua_isnumber(L, -1))
      v[ii] = lua_tonumber(L, -1);
    else
      WARNING("Element " << ii << " (" << ii+1 << ") of vector at stack index " << index << " is not a number");
    lua_pop(L, 1);
  }
  
  return v;
}

inline double lua_gettablenumber(lua_State *L, const char *key)
{
  int result = 0;
  
  lua_pushstring(L, key);
  lua_gettable(L, -2);
  
  if (lua_isnumber(L, -1))
    result = lua_tonumber(L, -1);
  else
    WARNING("Field '" << key << "' is not a number");
    
  lua_pop(L, 1);
  
  return result;
}

inline Vector lua_gettablevector(lua_State *L, const char *key)
{
  Vector result;
  
  lua_pushstring(L, key);
  lua_gettable(L, -2);
  
  if (lua_istable(L, -1))
    result = lua_tovector(L, -1);
  else
    WARNING("Field '" << key << "' is not a table");
    
  lua_pop(L, 1);
  
  return result;
}

}

#endif /* GRL_LUA_UTILS_H_ */
