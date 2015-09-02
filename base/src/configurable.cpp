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

#include <algorithm>
#include <grl/configurable.h>

using namespace grl;

unsigned char grl::grl_log_verbosity__ = 3;
const char *grl::grl_log_levels__[] = {"\x1B[31m\x1B[1m(ERR)\x1B[0m", "\x1B[33m\x1B[1m(WRN)\x1B[0m", "\x1B[34m\x1B[1m(NTC)\x1B[0m", "\x1B[32m\x1B[1m(INF)\x1B[0m", "\x1B[0m\x1B[1m(DBG)\x1B[0m", "\x1B[0m(CRL)"};

Configurable *YAMLConfigurator::load(const YAML::Node &node, Configuration *config, const std::string &path)
{
  Configurable *obj=NULL;
  Configuration objconfig;
  
  if (!node.IsMap())
  {
    ERROR("Can only load YAML maps");
    return NULL;
  }
    
  if (node["type"])
  {
    obj = ConfigurableFactory::create(node["type"].as<std::string>());
    
    if (!obj)
    {
      ERROR("Object " << path << " requested unknown type " << node["type"]);
      return NULL;
    }
    
    if (!path.empty())
      obj->setPath(path.substr(0, path.size()-1));
    objects_.push_back(obj);
    
    INFO(obj->path() << ": " << obj->d_type() << " (" << obj << ")");
  }
    
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
  {
    std::string key, value;
    key = it->first.as<std::string>();
    
    if (key == "type")
      continue;
      
    if (it->second.IsMap())
    {
      Configuration subcfg(objconfig);
      Configurable *subobj = load(it->second, &subcfg, path + key + "/");
      
      if (!subobj)
      {
        safe_delete(&obj);
        return NULL;
      }
      
      objconfig.set(key, subobj);
      references_.set(path + key, subobj);
    }
    else
    {
      value = toString(it->second);
      
      if (value.size() >= 5 && value.substr(value.size()-5) == ".yaml")
      {
        Configuration subcfg(objconfig);
        
        if (!file_.empty())
        {
          size_t pathsep = file_.back().find_last_of('/');
          if (pathsep != std::string::npos)
            value = file_.back().substr(0, pathsep+1) + value;
        }
        
        Configurable *subobj = load(value, &subcfg, path + key + "/");
        
        if (!subobj)
        {
          ERROR("Subconfiguration " << value << " did not yield an object");
          safe_delete(&obj);
          return NULL;
        }
            
        objconfig.set(key, subobj);
        references_.set(path + key, subobj);
      }
      else
      {
        value = parse(value);
        INFO(path << key << ": " << value);
      
        objconfig.set(key, value);
        references_.set(path + key, value);
      }
    }
  }
  
  if (obj)
  {
    ConfigurationRequest request;
    obj->request("", &request);
    
    // Check configuration against request
    for (size_t ii=0; ii < request.size(); ++ii)
    {
      std::string key = request[ii].name;
    
      if (request[ii].mutability == CRP::Provided)
      {
        // Do nothing here. It's not really a request.
      }
      else if (objconfig.has(key))
      {
        std::string value = objconfig[key].str();
        
        if (!validate(obj, path + key, value, request[ii]))
          return NULL;
      }
      else if (request[ii].optional)
      {
        INFO(path << key << ": " << request[ii].value << " (default)");
        objconfig.set(key, request[ii].value);
      }
      else
      {
        ERROR("Required parameter " << path << key << " is undefined");
        return NULL;
      }
    }

    // Check for unused variables
    for (Configuration::MapType::const_iterator ii=objconfig.parameters().begin(); ii != objconfig.parameters().end(); ++ii)
    {
      bool found=false;

      for (size_t jj=0; jj < request.size(); ++jj)
        if (ii->first == request[jj].name)
        {
          found = true;
          break;
        }

      if (!found)
        WARNING("Spurious parameter " << path << ii->first);
    }
  
    DEBUG("Configuring " << obj->d_type());
    try
    {
      obj->configure(objconfig);
    }
    catch (std::exception &e)
    {
      ERROR(e.what());
      return NULL;
    }
  }
    
  for (Configuration::MapType::const_iterator ii=objconfig.parameters().begin(); ii != objconfig.parameters().end(); ++ii)
    references_.set(path + ii->first, ii->second->str());
  
  config->merge(objconfig);
  return obj;
}      

void YAMLConfigurator::reconfigure(const Configuration &config, const std::string &action)
{
  Configuration base_message, message;
  Configurable *prev_object = NULL;
  
  // Prepare base message.
  if (!action.empty())
    base_message.set("action", action);
  message = base_message;

  for (Configuration::MapType::const_iterator ii=config.parameters().begin(); ii != config.parameters().end(); ++ii)
  {
    const std::string &key = ii->first;
    const std::string &value = ii->second->str();
    
    size_t seppos = key.rfind('/');
    std::string path      = key.substr(0, seppos),
                parameter = key.substr(seppos+1);
                
    if (references_.has(path))
    {
      Configurable *object = (Configurable*)references_[path].ptr();
      
      if (object)
      {
        if (prev_object && object != prev_object && !message.parameters().empty())
        {
          prev_object->reconfigure(message);
          message = base_message;
        }
        prev_object = object;
        
        if (action.empty())
        {
          // Straight up reconfiguration, check parameters
          ConfigurationRequest request;
          object->request("", &request);
          
          bool requested = false;
          for (size_t jj=0; jj < request.size(); ++jj)
          {
            if (request[jj].name == parameter)
            {
              if (request[jj].mutability == CRP::Online)
              {
                if (validate(object, key, value, request[jj]))
                {
                  INFO(key << ": " << request[jj].value << " -> " << value);
        
                  message.set(parameter, value);
                }
              }
              else
                WARNING("Cannot reconfigure '" << key << "': not a reconfigurable parameter.");
                
              requested = true;
              break;
            }
          }
          
          if (!requested)
            WARNING("Cannot reconfigure '" << key << "': no such parameter.");
        }
        else
        {
          // Not a reconfiguration, but a message. Don't check parameters.
          message.set(parameter, value);
        }
      }
      else
        WARNING("Cannot reconfigure '" << path << "': not a configurable object.");
    }
    else
      WARNING("Cannot reconfigure '" << path << "': no such object.");
  }

  if (prev_object && !message.parameters().empty())
    prev_object->reconfigure(message);
}

std::string YAMLConfigurator::parse(const std::string &value) const
{
  std::string v = value, id, expv;
  
  // Resolve references
  for (size_t ii=0; ii < v.size(); ++ii)
  {
    if (!isalnum(v[ii]) && v[ii] != '/' && v[ii] != '_')
    {
      if (references_.has(id))
        expv.insert(expv.size(), references_[id].str());
      else
        expv.insert(expv.size(), id);
    
      id.clear();
      expv.push_back(v[ii]);
    }
    else
      id.push_back(v[ii]);
  }
  
  if (references_.has(id))
    expv.insert(expv.size(), references_[id].str());
  else
    expv.insert(expv.size(), id);

  // Do some light math
  size_t c;
  while ((c=expv.find('+')) != std::string::npos)
  {
    std::string left, right;
    size_t start=c, end=c+1;
    
    // Right
    for (size_t ii=c+1; ii < expv.size() && expv[ii] != '+'; ++ii, ++end)
      right.push_back(expv[ii]);

    // Left
    for (int ii=c-1; ii >= 0 && expv[ii] != '+'; --ii, --start)
      left.push_back(expv[ii]);

    std::reverse(left.begin(), left.end());
    
    if (left.empty() || right.empty())
      return value;
      
    // Parse values
    std::istringstream issa, issb;
    Vector a, b, c;

    issa.str(left);  issa >> a;
    issb.str(right); issb >> b;
    
    // Perform operation
    if (a.size() == 1 && b.size() == 1) c = a + b;
    else                                c = extend(a, b);
    
    // Replace in original string
    std::ostringstream oss;
    if (c.size() == 1) oss << c[0];
    else               oss << c;
    
    expv.replace(start, end-start, oss.str());
  }
  
  return expv;
}

bool YAMLConfigurator::validate(Configurable *obj, const std::string &key, const std::string &value, const CRP &crp)
{
  std::string type, role;
  CRP::split(crp.type, &type, &role);

  if (type == "int")
  {
    int i;
    if (convert(value, &i))
    {
      if (i < crp.min || i > crp.max)
      {
        ERROR("Parameter " << key << " ('" << value << "') is out of range " << crp.min << " - " << crp.max);
        return false;
      }
    }
    else
    {
      ERROR("Parameter " << key << " ('" << value << "') should be an integer");
      return false;
    }
  }
  else if (type == "double")
  {
    double d;
    if (convert(value, &d))
    {
      if (d < crp.min || d > crp.max)
      {
        ERROR("Parameter " << key << " ('" << value << "') is out of range " << crp.min << " - " << crp.max);
        return false;
      }
    }
    else
    {
      ERROR("Parameter " << key << " ('" << value << "') should be a floating point value");
      return false;
    }
  }
  else if (type == "vector")
  {
    Vector v;
    if (!convert(value, &v))
    {
      ERROR("Parameter " << key << " ('" << value << "') should be a vector");
      return false;
    }
  }
  else if (type == "string")
  {
    if (!crp.options.empty())
    {
      bool found=false;
      for (size_t jj=0; jj < crp.options.size(); ++jj)
        if (value == crp.options[jj])
          found = true;
          
      if (!found)
      {
        std::cerr << "Parameter " << key << " ('" << value << "') should be one of {";
        for (size_t jj=0; jj < crp.options.size(); ++jj)
        {
          std::cerr << crp.options[jj];
          if (jj < crp.options.size() - 1)
            std::cerr << ", ";
        }
        std::cerr << "}" << std::endl;                
        return false;
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
        ERROR("Parameter " << key << " should subclass " << type);
        return false;
      }
      
      obj->children_.push_back(subobj);
    }
    else
    {
      ERROR("Parameter " << key << " ('" << value << "') should be an object (of type " << type << ")");
      return false;
    }
  }
  
  return true;
}
