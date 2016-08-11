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

/// Write out YAML node as string. 
std::string YAMLToString(const YAML::Node &node)
{
  if (node.IsScalar())
  {
    return node.as<std::string>();
  }
  else if (node.IsSequence())
  {
    std::string value;
  
    /* Ugly hack to convert sequence back to a string */
    std::stringstream ss;
    ss << "[ ";

    for (size_t ii=0; ii < node.size(); ++ii)
    {
      value = node[ii].as<std::string>();
      ss << value;
      if (ii < node.size()-1)
        ss << ", ";
    }
    ss << " ]";
    value = ss.str();

    return value;
  }
  else
    throw Exception("Unsupported YAML node type");
}

Configurator *grl::loadYAML(const std::string &file, const std::string &element, Configurator *parent)
{
  return loadYAML(file, element, parent, YAML::LoadFile(file.c_str()));
}

Configurator *grl::loadYAML(const std::string &file, const std::string &element, Configurator *parent, const YAML::Node &node)
{
  std::string path = element;
  if (parent)
    path = parent->path() + "/" + path;

  if (node.IsMap())
  {
    Configurator *cfg;
  
    if (node["type"])
    {
      // Object
      TRACE(path << ": object of type " << node["type"].as<std::string>());
      cfg = new ObjectConfigurator(element, node["type"].as<std::string>(), parent);
    }
    else
    {
      // Subhierarchy
      cfg = new Configurator(element, parent);
    }
    
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
    {
      std::string key = it->first.as<std::string>();
      
      if (key == "type")
        continue;
        
      loadYAML(file, key, cfg, it->second);
    }
    
    return cfg;
  }
  else
  {
    std::string value = YAMLToString(node);
    
    if (value.size() >= 5 && value.substr(value.size()-5) == ".yaml")
    {
      // Subhierarchy from file
      if (!file.empty())
      {
        size_t pathsep = file.find_last_of('/');
        if (pathsep != std::string::npos)
          value = file.substr(0, pathsep+1) + value;
      }
      
      TRACE(path << ": reference to file " << value);
      return loadYAML(value, element, parent);
    }
    else
    {
      // Parameter
      TRACE(path << ": " << value);
      return new ParameterConfigurator(element, value, parent);
    }
  }
}      

/// *** ParameterConfigurator ***

Configurator *ParameterConfigurator::resolve(const std::string &id)
{
  Configurator *reference = Configurator::find(id);
  if (!reference)
    reference = Configurator::find("/" + id);
  return reference;
}
 
std::string ParameterConfigurator::localize(const std::string &id) const
{
  if (id.empty())
    return id;

  const Configurator *reference = resolve(id);
  if (reference)
  {
    std::string fullpath = reference->path(), ownpath = path(), newpath;
    size_t ii;
    
    // Find first path difference
    for (ii=0; ii < fullpath.size() && ii < ownpath.size() && ownpath[ii] == fullpath[ii]; ++ii);
    
    // Make sure we're on a full folder name
    while (ownpath[ii] != '/')
      ii--;
      
    // Localized path ends with other's path from this point onwards
    newpath = fullpath.substr(ii+1);

    // Add ../ for every folder on our own path until the end
    for (; ii < ownpath.size(); ++ii)
      if (ownpath[ii] == '/')
        newpath = "../" + newpath;
        
    return newpath;
  }
  else
    return id;
}

std::string ParameterConfigurator::str() const
{
  std::string v = value_, id, expv;
  
  // Resolve references
  for (size_t ii=0; ii < v.size(); ++ii)
  {
    if (!isalnum(v[ii]) && v[ii] != '/' && v[ii] != '_' && v[ii] != '.')
    {
      if (!id.empty())
      {
        const Configurator *reference = resolve(id);

        if (reference)
          expv.insert(expv.size(), reference->str());
        else
          expv.insert(expv.size(), id);

        id.clear();
      }
      expv.push_back(v[ii]);
    }
    else
      id.push_back(v[ii]);
  }
  
  if (!id.empty())
  {
    const Configurator *reference = resolve(id);

    if (reference)
      expv.insert(expv.size(), reference->str());
    else
      expv.insert(expv.size(), id);
  }

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
      break;
      
    // Parse values
    std::istringstream issa, issb;
    std::vector<double> a_in, b_in, c_out;

    issa.str(left);  issa >> a_in;
    issb.str(right); issb >> b_in;
    
    Vector a, b, c;
    toVector(a_in, a);
    toVector(b_in, b);
    
    // Perform operation
    if (a.size() == 1 && b.size() == 1) c = a + b;
    else                                c = extend(a, b);
    
    fromVector(c, c_out);
    
    // Replace in original string
    std::ostringstream oss;
    if (c.size() == 1) oss << c_out[0];
    else               oss << c_out;
    
    expv.replace(start, end-start, oss.str());
  }
  
  return expv;
}

Configurable *ParameterConfigurator::ptr()
{
  if (!value_.empty())
  {
    Configurator *reference = resolve(value_);
    if (reference)
      return reference->ptr();
  }
  
  return NULL;
}

Configurator *ParameterConfigurator::find(const std::string &path)
{
  if (!value_.empty())
  {
    Configurator *reference = resolve(value_);
    
    if (reference)
      return reference->find(path);
  }

  return Configurator::find(path);
}

ParameterConfigurator *ParameterConfigurator::instantiate(Configurator *parent) const
{
  if (!parent)
    parent = parent_;

  std::string v = value_, id, expv;

  // Make references local
  for (size_t ii=0; ii < v.size(); ++ii)
    if (!isalnum(v[ii]) && v[ii] != '/' && v[ii] != '_' && v[ii] != '.')
    {
      expv.insert(expv.size(), localize(id));
      id.clear();
      expv.push_back(v[ii]);
    }
    else
      id.push_back(v[ii]);
  
  expv.insert(expv.size(), localize(id));
  
  return new ParameterConfigurator(element_, expv, parent);
}

bool ParameterConfigurator::validate(const CRP &crp) const
{
  const Configurator *reference;

  if (!value_.empty() && (reference = resolve(value_)))
  {
    return reference->validate(crp);
  }
  else
  {
    std::string type, role, value = str();
    CRP::split(crp.type, &type, &role);

    if (type == "int")
    {
      int i;
      if (convert(value, &i))
      {
        if (i < crp.min || i > crp.max)
        {
          ERROR("Parameter " << path() << " ('" << value << "') is out of range " << crp.min << " - " << crp.max);
          return false;
        }
      }
      else
      {
        ERROR("Parameter " << path() << " ('" << value << "') should be an integer");
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
          ERROR("Parameter " << path() << " ('" << value << "') is out of range " << crp.min << " - " << crp.max);
          return false;
        }
      }
      else
      {
        ERROR("Parameter " << path() << " ('" << value << "') should be a floating point value");
        return false;
      }
    }
    else if (type == "vector")
    {
      Vector v;
      if (!convert(value, &v))
      {
        ERROR("Parameter " << path() << " ('" << value << "') should be a vector");
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
          std::cerr << "Parameter " << path() << " ('" << value << "') should be one of {";
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
    else if (value != "0")
    {
      ERROR("Parameter " << path() << " ('" << value << "') should be an object");
      return false;
    }
  }
    
  return true;
}

void ParameterConfigurator::reconfigure(const Configuration &config, bool recursive)
{
  Configurator *reference;

  if (!value_.empty() && (reference = resolve(value_)))
    reference->reconfigure(config, recursive);

  return;
}

/// *** ObjectConfigurator ***

ObjectConfigurator::~ObjectConfigurator()
{
  // Avoid double free by Configurable in turn.
  if (object_)
  {
    CRAWL(path() << ": Deleting associated object " << object_ << " (type " << object_->d_type() << ")");
  
    object_->detach();
    safe_delete(&object_);
  }
}

ObjectConfigurator* ObjectConfigurator::instantiate(Configurator *parent) const
{
  if (!parent)
    parent = parent_;

  // Create object
  ObjectConfigurator *oc = new ObjectConfigurator(element_, type_, parent);
  oc->object_ = ConfigurableFactory::create(type_);
  
  if (!oc->object_)
  {
    ERROR("Unable to create object of type " << type_);
    return NULL;
  }

  Configuration config;
  ConfigurationRequest request;
  oc->object_->attach(oc);
  oc->object_->request("", &request);

  // Instantiate and validate configuration parameters
  for (size_t ii=0; ii < request.size(); ++ii)
  {
    std::string key = request[ii].name;
  
    if (request[ii].mutability != CRP::Provided)
    {
      // Find matching Configurator
      for (ConfiguratorList::const_iterator cc=children_.begin(); cc != children_.end(); ++cc)
      {
        if ((*cc)->element() == key)
        {
          // Instantiate
          Configurator *nc = (*cc)->instantiate(oc);
          if (!nc)
            return NULL;
          
          // Validate against request
          if (!nc->validate(request[ii]))
            return NULL;

          INFO(path() << "/" << key << ": " << nc->str());
          config.set(key, nc->str());
        }
      }
      
      // Check for unspecified parameters
      if (!config.has(key))
      {
        if (request[ii].optional)
        {
          new ParameterConfigurator(key, request[ii].value, oc);
          
          INFO(path() << "/" << key << ": " << request[ii].value << " (default)");
          config.set(key, request[ii].value);
        }
        else
        {
          ERROR("Required parameter " << path() << "/" << key << " is undefined");
          return NULL;
        }
      }
    }
  }

  // Configure object
  oc->object_->configure(config);
  
  // Add provided parameters to instantiated tree
  for (size_t ii=0; ii < request.size(); ++ii)
  {
    std::string key = request[ii].name, type, role;
    CRP::split(request[ii].type, &type, &role);
  
    if (request[ii].mutability == CRP::Provided)
    {
      INFO(path() << "/" << key << ": " << config[key].str() << " (provided)");
    
      if (type == "int" || type == "double" || type == "vector" || type == "string")
      {
        new ParameterConfigurator(key, config[key].str(), oc);
      }
      else
      {
        ObjectConfigurator *nc = new ObjectConfigurator(key, request[ii].type, oc);
        nc->attach((Configurable*)config[key].ptr());
      }
    }
  }
  
  return oc;
}

bool ObjectConfigurator::validate(const CRP &crp) const
{
  std::string type, role;
  CRP::split(crp.type, &type, &role);

  if (object_)
  {
    if (object_->d_type().substr(0, type.size()) != type)
    {
      ERROR("Parameter " << path() << " should subclass " << type);
      return false;
    }
  }
  else
  {
    ERROR("Parameter " << path() << " is not an instantiated object");
    return false;
  }
  
  return true;
}

void ObjectConfigurator::reconfigure(const Configuration &config, bool recursive)
{
  if (recursive)
    Configurator::reconfigure(config, recursive);

  if (!config.has("action"))
  {
    // Straight up reconfiguration, check parameters
    ConfigurationRequest request;
    object_->request("", &request);
    
    for (Configuration::MapType::const_iterator ii=config.parameters().begin(); ii != config.parameters().end(); ++ii)
    {
      const std::string &key = ii->first;
      const std::string &value = ii->second->str();
      
      bool requested = false;
      for (size_t jj=0; jj < request.size(); ++jj)
      {
        if (request[jj].name == key)
        {
          if (request[jj].mutability == CRP::Online)
          {
            ParameterConfigurator pc(key, value);
          
            if (!pc.validate(request[jj]))
              return;
            
            INFO(pc.path() << ": " << request[jj].value << " -> " << value);
              
            /// TODO: Update tree
          }
          else
            WARNING("Cannot reconfigure '" << key << "': not a reconfigurable parameter.");
            
          requested = true;
          break;
        }
      }
      
      if (!requested && !recursive)
        WARNING("Cannot reconfigure '" << key << "': no such parameter.");
    }
  }
  
  object_->reconfigure(config);
}
