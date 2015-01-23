/** \file configurable.cpp
 * \brief YAML configurator.
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

#include <grl/configurable.h>

using namespace grl;

Configurable *YAMLConfigurator::load(const YAML::Node &node, Configuration *config, const std::string &path)
{
  Configurable *obj=NULL;
  
  if (!node.IsMap())
    throw Exception("Can only load YAML maps");
    
  if (node["type"])
  {
    obj = ConfigurableFactory::create(node["type"].as<std::string>());
    
    if (!obj)
    {
      std::cerr << "Object " << path << " requested unknown type " << node["type"] << std::endl;
      return NULL;
    }
  }
    
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
  {
    std::string key, value;
    key = it->first.as<std::string>();
    
    if (key == "type")
      continue;
      
    if (it->second.IsMap())
    {
      Configuration subcfg(*config);
      
      Configurable *subobj = load(it->second, &subcfg, path + key + "/");
      
      if (!subobj)
      {
        safe_delete(&obj);
        return NULL;
      }
      
      std::cout << path << key << ": " << subobj << " (type " << subobj->d_type() << ") " << std::endl;
              
      config->set(key, subobj);
      references_.set(path + key, subobj);
    }
    else
    {
      value = toString(it->second);
      
      if (references_.has(value))
      {
        std::cout << path << key << ": " << references_[value].str() << " (from " << value << ")" << std::endl;
        value = references_[value].str();
      }
      else
        std::cout << path << key << ": " << value << std::endl;
      
      config->set(key, value);
      references_.set(path + key, value);
    }
  }
  
  ConfigurationRequest request;
  obj->request(&request);
  
  // Check configuration against request
  for (size_t ii=0; ii < request.size(); ++ii)
  {
    std::string key = request[ii].name;
    std::string type = request[ii].type;
  
    if (config->has(key))
    {
      std::string value = (*config)[key].str();
      
      if (type == "int")
      {
        int i;
        if (convert(value, &i))
        {
          if (i < request[ii].min || i > request[ii].max)
          {
            std::cerr << "Parameter " << path << key << " ('" << value << "') is out of range " << request[ii].min << " - " << request[ii].max << std::endl;
            return NULL;
          }
        }
        else
        {
          std::cerr << "Parameter " << path << key << " ('" << value << "') should be an integer" << std::endl;
          return NULL;
        }
      }
      else if (type == "double")
      {
        double d;
        if (convert(value, &d))
        {
          if (d < request[ii].min || d > request[ii].max)
          {
            std::cerr << "Parameter " << path << key << " ('" << value << "') is out of range " << request[ii].min << " - " << request[ii].max << std::endl;
            return NULL;
          }
        }
        else
        {
          std::cerr << "Parameter " << path << key << " ('" << value << "') should be a floating point value" << std::endl;
          return NULL;
        }
      }
      else if (type == "vector")
      {
        Vector v;
        if (!convert(value, &v))
        {
          std::cerr << "Parameter " << path << key << " ('" << value << "') should be a vector" << std::endl;
          return NULL;
        }
      }
      else if (type == "string")
      {
        if (!request[ii].options.empty())
        {
          bool found=false;
          for (size_t jj=0; jj < request[ii].options.size(); ++jj)
            if (value == request[ii].options[jj])
              found = true;
              
          if (!found)
          {
            std::cerr << "Parameter " << path << key << " ('" << value << "') should be one of {";
            for (size_t jj=0; jj < request[ii].options.size(); ++jj)
            {
              std::cerr << request[ii].options[jj];
              if (jj < request[ii].options.size() - 1)
                std::cerr << ", ";
            }
            std::cerr << "}" << std::endl;                
            return NULL;
          }
        }
      }
      else
      {
        if (value.substr(0, 2) == "0x")
        {
          Configurable *subobj = (Configurable*)strtol(value.c_str(), NULL, 0);
          
          std::string t = subobj->d_type();
      
          if (subobj->d_type().substr(0, type.size()) != type)
          {
            std::cerr << "Parameter " << path << key << " should subclass " << type << std::endl;
            return NULL;
          }
        }
        else
        {
          std::cerr << "Parameter " << path << key << " ('" << value << "') should be an object (of type " << type << ")" << std::endl;
          return NULL;
        }
      }
    }
    else if (type != "int" && type != "double" && type != "vector" && type != "string")
    {
      std::cerr << "Object parameter " << path << key << " is undefined" << std::endl;
      return NULL;
    }
    else
    {
      std::cout << path << key << ": " << request[ii].value << " (default)" << std::endl;
      config->set(key, request[ii].value);
    }
  }
  
  if (obj)
  {
    std::cout << "Configuring " << obj->d_type() << std::endl;
    obj->configure(*config);
  }
    
  for (Configuration::MapType::const_iterator ii=config->parameters().begin(); ii != config->parameters().end(); ++ii)
    references_.set(path + ii->first, ii->second->str());
    
  return obj;
}      
