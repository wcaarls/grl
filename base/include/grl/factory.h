/** \file factory.h
 * \brief Macros to create object factories.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
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

#include <string>
#include <map>

/// Declare an object factory.
/**
 * Typically called after declaring the class.
 */
#define DECLARE_FACTORY(x)                            \
class x ## Factory                                    \
{                                                     \
  private:                                            \
    typedef std::map<std::string, x ## Factory*> Map; \
    static Map &factories();                          \
                                                      \
  protected:                                          \
    x ## Factory(std::string name);                   \
    virtual x *create() = 0;                          \
                                                      \
  public:                                             \
    static x *create(std::string name);               \
};

/// Define an object factory.
/**
 * Typically called in the source file that defines the class members.
 */
#define DEFINE_FACTORY(x)                             \
x ## Factory::Map &x ## Factory::factories()          \
{                                                     \
  static Map factories_;                              \
  return factories_;                                  \
}                                                     \
                                                      \
x ## Factory::x ## Factory(std::string name)          \
{                                                     \
  std::cout << "registering " << name << std::endl;   \
  factories()[name] = this;                           \
}                                                     \
                                                      \
x* x ## Factory::create(std::string name)             \
{                                                     \
  if (!factories().count(name))                       \
    return NULL;                                      \
                                                      \
  x *obj = factories()[name]->create();               \
                                                      \
  return obj;                                         \
}

/// Register a derived class with an object factory
/**
 * Typically called in the source file that defines the derived class members.
 */
#define REGISTER_FACTORY(x, subx, name)               \
static class subx ## Factory : public x ## Factory    \
{                                                     \
  public:                                             \
    subx ## Factory() : x ## Factory(name) { }        \
    virtual x *create()                               \
    {                                                 \
      return new subx();                              \
    }                                                 \
} subx ## _factory;
