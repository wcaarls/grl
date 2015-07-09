/** \file configuration.h
 * \brief Parameters for object configuration.
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

#ifndef GRL_CONFIGURATION_H_
#define GRL_CONFIGURATION_H_

#include <exception>
#include <string>
#include <map>
#include <vector>

#include <sstream>
#include <fstream>

#include <grl/vector.h>

namespace grl {

template <typename T> struct type_name {
    static const char *name;
};

#define DECLARE_TYPE_NAME(x) template<> const char *type_name<x>::name = #x;

/// General exception.
class Exception : public std::exception
{
  protected:
    std::string what_;

  public:
    Exception(std::string what) throw()
    {
      what_ = what;
    }
    virtual ~Exception() throw () { }

    virtual const char* what() const throw()
    {
      return what_.c_str();
    }
};

/// Bad configuration parameter exception.
class bad_param : public Exception
{
  public:
    bad_param(std::string which) throw() : Exception("Parameter '" + which + "' has an illegal value") { }
};

/// Configuration parameter.
class ConfigurationParameter
{
  protected:
    std::string value_;

  public:
    template <class T>
    ConfigurationParameter(T value)
    {
      std::ostringstream oss;
      oss << value;
      value_ = oss.str();
    }

    ConfigurationParameter(const ConfigurationParameter &other)
    {
      value_ = other.str();
    }
    
    template <class T>
    bool get(T &value) const
    {
      if (value_.empty())
      {
        value = T();
        return true;
      }
      else
      {
        std::istringstream iss(value_);
        iss >> value;
        return !iss.fail();
      }
    }

    template<class T>
    operator T() const
    {
      T value;
      if (!get(value))
        throw Exception("Parameter type mismatch while converting '" + value_ + "' to " + type_name<T>::name);
      return value;
    }
    
    std::string str() const
    {
      return value_;
    }
    
    void *ptr() const
    {
      void *value;
      if (!get(value))
        throw Exception("Parameter type mismatch while converting '" + value_ + "' to pointer");
      return value;
    }
};

/// Set of configuration parameters.
class Configuration
{
  public:
    typedef std::map<std::string, const ConfigurationParameter*> MapType;

  protected:
    MapType parameters_;

  public:
    Configuration()
    {
    }
  
    Configuration(const Configuration &other)
    {
      for (MapType::const_iterator ii=other.parameters_.begin(); ii != other.parameters_.end(); ++ii)
        parameters_[ii->first] = new ConfigurationParameter(*ii->second);
    }
  
    ~Configuration()
    {
      for (MapType::iterator ii=parameters_.begin(); ii != parameters_.end(); ++ii)
        delete ii->second;
    }
    
    Configuration &operator=(const Configuration &other)
    {
      if (&other != this)
      {
        for (MapType::iterator ii=parameters_.begin(); ii != parameters_.end(); ++ii)
          delete ii->second;
        parameters_.clear();
      
        merge(other);
      }
        
      return *this;
    }
    
    void merge(const Configuration &other)
    {
      for (MapType::const_iterator ii=other.parameters_.begin(); ii != other.parameters_.end(); ++ii)
        parameters_[ii->first] = new ConfigurationParameter(*ii->second);
    }

    bool has (const std::string &key) const
    {
      return parameters_.count(key) > 0;
    }

    template<class T>
    bool get (const std::string &key, T &value) const
    {
      return get(key, value, value);
    }

    template<class T>
    bool get (const std::string &key, T &value, const T &dflt) const
    {
      if (has(key))
      {
        if (!parameters_.at(key)->get(value))
          throw Exception("Parameter type mismatch for '" + key + "'");

        return true;
      }
      else
      {
        value = dflt;
        return false;
      }
    }

    bool get (const std::string &key, std::string &value, const char *dflt) const
    {
      if (has(key))
      {
        if (!parameters_.at(key)->get(value))
          throw Exception("Parameter type mismatch for '" + key + "'");

        return true;
      }
      else
      {
        value = dflt;
        return false;
      }
    }

    const ConfigurationParameter& operator[](const std::string &key) const
    {
      if (!has(key))
        throw Exception("Parameter '" + key + "' not set");

      return *parameters_.at(key);
    }

    template<class T>
    void set (const std::string &key, const T &value)
    {
      if (has(key))
        delete parameters_[key];
      parameters_[key] = new ConfigurationParameter(value);
    }
    
    const MapType &parameters() const
    {
      return parameters_;
    }
};

}

#endif /* GRL_CONFIGURATION_H_ */
