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
  // Case 1: key:value pair
  std::string key = file;
  char *value = strchr((char*)key.c_str(), ':');
  if (value)
  {
    *(value++) = '\0';
    
    if (!parent)
      parent = new Configurator();
    
    loadYAML("", key.c_str(), parent, YAML::Load(value));
    
    return parent;
  }
  
  // Case 2: merging
  if (element.empty() && parent)
  {
    YAML::Node node = YAML::LoadFile(file.c_str());
    
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
      loadYAML(file, it->first.as<std::string>(), parent, it->second);
      
    return parent;
  }

  // Case 3: loading
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
      std::string type = node["type"].as<std::string>(), newtype;
      
      // Normalize type
      ConfigurableFactory::Map factories = ConfigurableFactory::factories();
      if (!factories.count(type))
      {
        size_t count=0;
        for (ConfigurableFactory::Map::iterator ii=factories.begin(); ii != factories.end(); ++ii)
          if (ii->first.size() > type.size())
            if (!ii->first.compare(ii->first.size()-type.size(), std::string::npos, type))
            {
              CRAWL(path << ": " << type << " expands to " << ii->first);
              newtype = ii->first;
              count++;
            }
            
        if (count > 1)
          WARNING(path << ": " << type << " does not specify a unique type. Expanded to " << newtype);
          
        if (newtype.size())
          type = newtype;
      }
      
      TRACE(path << ": object of type " << type);
      cfg = new ObjectConfigurator(element, type, parent);
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
  else if (node.IsSequence() && node.size() > 0 &&
           std::isnan(node[0].as<double>(std::numeric_limits<double>::quiet_NaN())))
  {
    ListConfigurator *cfg = new ListConfigurator(element, parent);
    
    for (size_t ii=0; ii < node.size(); ++ii)
    {
      std::ostringstream oss;
      oss << ii;
      loadYAML(file, oss.str(), cfg, node[ii]);
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

/// *** ListConfigurator ***

ListConfigurator *ListConfigurator::instantiate(Configurator *parent) const
{
  if (!parent)
    parent = parent_;

  ListConfigurator *cfg = new ListConfigurator(element_, parent);

  for (ConfiguratorList::const_iterator ii=children_.begin(); ii != children_.end(); ++ii)
    if (!(*ii)->instantiate(cfg))
      return NULL;
      
  // Populate ConfigurableList in instantiated configurator
  for (ConfiguratorList::const_iterator ii=cfg->children_.begin(); ii != cfg->children_.end(); ++ii)
    cfg->object_.push_back((*ii)->ptr());

  return cfg;
}

bool ListConfigurator::validate(const CRP &crp) const
{
  // Check for list type
  if (crp.type.empty() || crp.type.front() != '[' || crp.type.back() != ']')
  {
    ERROR("Parameter " << path() << " should be a single object");
    return false;
  }

  // Adjust request type to remove brackets
  CRP ccrp = crp;
  ccrp.type = ccrp.type.substr(1, ccrp.type.size()-2);
  
  // Validate all children with same CRP
  for (ConfiguratorList::const_iterator ii=children_.begin(); ii != children_.end(); ++ii)
    if (!(*ii)->validate(ccrp))
      return false;
      
  return true;
}

/// *** ReferenceConfigurator ***

std::string ReferenceConfigurator::reference_path() const
{
  std::string path = reference_->reference_path();
  if (!path_.empty())
    path = path + "/" + path_;
  return path;
}

std::string ReferenceConfigurator::str() const
{
  Configurator *ref = reference_->find(path_);
  if (ref)
    return ref->str();
  else
    return "";
}

Configurable *ReferenceConfigurator::ptr()
{
  Configurator *ref = reference_->find(path_);
  if (ref)
    return ref->ptr();
  else
    return NULL;
}

Configurator *ReferenceConfigurator::find(const std::string &path)
{
  if (!path_.empty())
    return reference_->find(path_ + "/" + path);
  else 
    return reference_->find(path);
}

ReferenceConfigurator *ReferenceConfigurator::instantiate(Configurator *parent) const
{
  if (!parent)
  {
    ERROR(path() << ": Reference configurator must be instantiated as part of an object hierarchy");
    return NULL;
  }
  
  Configurator *ref = parent->find(relative_path(reference_path()).substr(3));
  
  if (!ref)
  {
    ERROR(path() << ": Reference configurator does not point to a valid object in the new hierarchy");
    return NULL;
  }
  
  // Rebase
  return new ReferenceConfigurator(element_, ref, "", parent);
}

bool ReferenceConfigurator::validate(const CRP &crp) const
{
  Configurator *ref = reference_->find(path_);
  if (ref)
    return ref->validate(crp);
  else
    return false;
}

void ReferenceConfigurator::reconfigure(const Configuration &config, bool recursive)
{
  Configurator *ref = reference_->find(path_);
  if (ref)
    ref->reconfigure(config, recursive);
}

/// *** ParameterConfigurator ***

bool ParameterConfigurator::isseparator(char c) const
{
  return c == ' ' || c == '\t' || c == '[' || c == ']' || c == '+' || c == ',';
}

Configurator *ParameterConfigurator::resolve(const std::string &id, Configurator *parent)
{
  if (parent)
  {
    Configurator *reference = parent->find(element_ + "/" + id);
    if (!reference)
      reference = parent->find("/" + id);
    return reference;
  }
  else
  {
    Configurator *reference = Configurator::find(id);
    if (!reference && parent_)
      reference = Configurator::find("/" + id);
    return reference;
  }
}
 
std::string ParameterConfigurator::localize(const std::string &id, const Configurator *parent) const
{
  if (id.empty())
    return id;
    
  const Configurator *reference = resolve(id, parent);
  
  if (reference)
    return relative_path(reference->path());
  else
    return id;
}

#define OPERATORS "+*"

std::string ParameterConfigurator::str() const
{
  std::string v = value_, id, expv;
  
  // Resolve references
  for (size_t ii=0; ii < v.size(); ++ii)
  {
    if (isseparator(v[ii]))
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
  while ((c=strcspn(expv.c_str(), OPERATORS)) < expv.size()) {
    std::string left, right;
    size_t start=c, end=c+1;
    int double_op = 0;

    // Recognize ++ for vector extension
    if (c+1 < expv.size() && expv[c] == expv[c+1])
    {
      double_op = 1;
      end++;
      c++;
    }

    // Right
    for (size_t ii=c+1; ii < expv.size() && !strchr(OPERATORS, expv[ii]); ++ii, ++end)
      right.push_back(expv[ii]);

    // Left
    for (int ii=c-1-double_op; ii >= 0 && !strchr(OPERATORS, expv[ii]); --ii, --start)
      left.push_back(expv[ii]);

    std::reverse(left.begin(), left.end());

    if (left.empty() || right.empty())
      break;

    // Parse values
    std::istringstream issx, issy;
    std::vector<double> x_in, y_in, z_out;

    issx.str(left);  issx >> x_in;
    issy.str(right); issy >> y_in;

    LargeVector x, y, z;
    toVector(x_in, x);
    toVector(y_in, y);

    // Perform operation
    switch (expv[c])
    {
      case '+':
        // TODO: Rationalize this to only extend vectors when using ++ operator
        if (x.size() == 1 && y.size() == 1 && !double_op) z = x + y;
        else                                              z = extend(x, y);
        break;
      case '*':
        if (x.size() == 1)             z = x[0] * y;
        else if (y.size() == 1)        z = x * y[0];
        else if (x.size() == y.size()) z = x * y;
        else
          ERROR("Cannot multiply " << x_in << " and " << y_in << ": vector size mismatch");
        break;
    }

    fromVector(z, z_out);

    // Replace in original string
    std::ostringstream oss;
    oss << std::setprecision(std::numeric_limits<double>::max_digits10);
    
    if (z.size() == 1) oss << z_out[0];
    else               oss << z_out;

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
  if (provided_)
    throw Exception(path() + ": tried to instantiate a provided parameter");

  if (!parent)
    parent = parent_;

  std::string v = value_, id, expv;

  // Make references local
  for (size_t ii=0; ii < v.size(); ++ii)
    if (isseparator(v[ii]))
    {
      expv.insert(expv.size(), localize(id, parent));
      id.clear();
      expv.push_back(v[ii]);
    }
    else
      id.push_back(v[ii]);
  
  expv.insert(expv.size(), localize(id, parent));
  
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
      LargeVector v;
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
  if (provided_)
    throw Exception(path() + ": tried to instantiate a provided object");

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
  for (ConfiguratorList::const_iterator cc=children_.begin(); cc != children_.end(); ++cc)
  {
    if ((*cc)->provided())
      continue;
  
    // Instantiate
    Configurator *nc = (*cc)->instantiate(oc);
    if (!nc)
      return NULL;
    
    TRACE(path() << "/" << (*cc)->element() << ": " << nc->str());

    // Find matching parameter request
    for (size_t ii=0; ii < request.size(); ++ii)
    {
      if (request[ii].mutability != CRP::Provided)
      {
        if ((*cc)->element() == request[ii].name)
        {
          // Validate against request
          if (!nc->validate(request[ii]))
            return NULL;

          config.set(request[ii].name, nc->str());
        }
      }
    }
  }
      
  // Check for unspecified parameters
  for (size_t ii=0; ii < request.size(); ++ii)
  {
    std::string key = request[ii].name;
    
    if (request[ii].mutability != CRP::Provided)
    {
      if (!config.has(key))
      {
        if (request[ii].optional)
        {
          new ParameterConfigurator(key, request[ii].value, oc);
          
          TRACE(path() << "/" << key << ": " << request[ii].value << " (default)");
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
      TRACE(path() << "/" << key << ": " << config[key].str() << " (provided)");
      if (type == "int" || type == "double" || type == "vector" || type == "string")
      {
        new ParameterConfigurator(key, config[key].str(), oc, true);
      }
      else
      {
        ObjectConfigurator *nc = new ObjectConfigurator(key, request[ii].type, oc, true);
        nc->attach((Configurable*)config[key].ptr());
      }
    }
  }
  
  return oc;
}

ObjectConfigurator &ObjectConfigurator::deepcopy(const Configurator &c)
{
  Configurator::deepcopy(c);

  if (!ptr() != !c.ptr())
    throw Exception("Deep copy requires similarly instantiated configuration trees at " + path());

  if (ptr() != c.ptr())
    ptr()->copy(*c.ptr());
  else
    ERROR("Deep copy encountered identical object at " + path());
  
  return *this;
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
            
            INFO(path() << "/" << pc.path() << ": " << request[jj].value << " -> " << value);
              
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
