/** \file configurable.h
 * \brief Configurable object definition.
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
#ifndef GRL_CONFIGURABLE_H_
#define GRL_CONFIGURABLE_H_

#include <limits.h>
#include <float.h>

#include <yaml-cpp/yaml.h>

#include <grl/utils.h>
#include <grl/configuration.h>
#include <grl/factory.h>

namespace grl {

/// Parameter requested by a Configurable object.
struct CRP
{
  typedef enum {Provided, System, Configuration, Online} Mutability;

  std::string name, type, description, value;
  Mutability mutability;
  double min, max;
  std::vector<std::string> options;
  bool optional;
  
  /// Request for a Configurable object.
  CRP(std::string _name, std::string _type, std::string _description,
      class Configurable *_value, bool _optional=false) :
    name(_name), type(_type), description(_description), mutability(Configuration), optional(_optional)
  {
    setValue(_value);
  }
 
  /// Request for an integer value.
  CRP(std::string _name, std::string _description,
      int _value, Mutability _mutability=Configuration, int _min=0, int _max=INT_MAX) :
    name(_name), type("int"), description(_description), mutability(_mutability), min(_min), max(_max), optional(true)
  {
    setValue(_value);
  }

  /// Request for an integer value with explicit type specification to set role.
  CRP(std::string _name, std::string _type, std::string _description,
      int _value, Mutability _mutability=Configuration, int _min=0, int _max=INT_MAX) :
    name(_name), type(_type), description(_description), mutability(_mutability), min(_min), max(_max), optional(true)
  {
    setValue(_value);
  }
  
  /// Request for a double value.
  CRP(std::string _name, std::string _description,
      double _value, Mutability _mutability=Configuration, double _min=0., double _max=1.) :
    name(_name), type("double"), description(_description), mutability(_mutability), min(_min), max(_max), optional(true)
  {
    setValue(_value);
  }
  
  /// Request for a double value with explicit type specification to set role.
  CRP(std::string _name, std::string _type, std::string _description,
      double _value, Mutability _mutability=Configuration, double _min=0., double _max=1.) :
    name(_name), type(_type), description(_description), mutability(_mutability), min(_min), max(_max), optional(true)
  {
    setValue(_value);
  }
  
  /// Request for a Vector value.
  CRP(std::string _name, std::string _description,
      Vector _value, Mutability _mutability=Configuration) :
    name(_name), type("vector"), description(_description), mutability(_mutability), optional(true)
  {
    setValue(_value);
  }

  /// Request for a Vector value with explicit type specification to set role.
  CRP(std::string _name, std::string _type, std::string _description,
      Vector _value, Mutability _mutability=Configuration) :
    name(_name), type(_type), description(_description), mutability(_mutability), optional(true)
  {
    setValue(_value);
  }

  /**
   * \brief Request for a string value or definition of a provided parameter.
   *
   * The two cases are disambiguated based on the _mutability parameter
   */
  CRP(std::string _name, std::string _description,
      std::string _value, Mutability _mutability=Configuration, 
      std::vector<std::string> _options=std::vector<std::string>()) :
    name(_name), type("string"), description(_description), value(_value), mutability(_mutability), options(_options), optional(true)
  { 
    // Provided parameters have the same signature as strings... Disambiguate.
    if (mutability == Provided)
    {
      type = description;
      description = value;
      value = "";
      options = std::vector<std::string>();
    }
  }
  
  /// Split type into base type and role.
  inline static void split(const std::string type, std::string *base, std::string *role)
  {
    if (type.find('.') != std::string::npos)
    {
      *base = type.substr(0, type.find('.'));
      *role = type.substr(base->size()+1);
    }
    else
    {
      *base = type;
      *role = "";
    }
  }
  
  protected:
    /// Sets the value member to a string description of the given parameter.
    template<class T>
    void setValue(const T& v)
    {
      std::ostringstream oss;
      oss << v;
      value = oss.str();
    }

    /// Sets the value member to a string description of the given vector parameter.
    void setValue(const Vector& v)
    {
      std::ostringstream oss;
      std::vector<double> v_out;
      fromVector(v, v_out);
      oss << v_out;
      value = oss.str();
    }
};

/// Set of requested parameters.
typedef std::vector<CRP> ConfigurationRequest;

/**
 * \brief Sets the type and description of a Configurable object.
 *
 * Obligatory for all concrete Configurable subclasses.
 */
#define TYPEINFO(t, d)\
    static std::string s_type() { return t; }\
    virtual std::string d_type() const { return t; }\
    virtual std::string description() const { return d; }

extern unsigned char grl_log_verbosity__;
extern const char *grl_log_levels__[];

/// Write a log message at the desired level.
#define GRLLOG(l, m) do { std::ostringstream __log_oss; __log_oss << m; log(l, __log_oss); } while (0)

#define ERROR(m)   GRLLOG(0, m) ///< Log an error message.
#define WARNING(m) GRLLOG(1, m) ///< Log a warning.
#define NOTICE(m)  GRLLOG(2, m) ///< Log a notable event.
#define INFO(m)    GRLLOG(3, m) ///< Log a general event.
#define TRACE(m)   GRLLOG(4, m) ///< Log throttled debugging information.
#define CRAWL(m)   GRLLOG(5, m) ///< Log unthrottled debugging information.

/// Global namespace log writer.
inline void log(unsigned char level, const std::ostringstream &oss)
{
  if (level <= grl_log_verbosity__)
  {
    if (level < 2)
      std::cerr << grl_log_levels__[level] << " " << oss.str() << "\x1B[0m" << std::endl;
    else
      std::cout << grl_log_levels__[level] << " " << oss.str() << "\x1B[0m" << std::endl;
  }
}

class Configurator
{
  public:
    typedef std::vector<Configurator*> ConfiguratorList;

  protected:
    Configurator *parent_;
    std::string element_;
    ConfiguratorList children_;
    
  public:
    Configurator(const std::string &element=std::string(), Configurator *parent=NULL) : parent_(parent), element_(element)
    {
      if (parent_)
      {
        for (ConfiguratorList::iterator ii=parent_->children_.begin(); ii != parent_->children_.end(); ++ii)
          if ((*ii)->element_ == element_)
          {
            CRAWL(path() << ": Cowardly refusing to overwrite parent's existing child");
            return;
          }
          
        parent_->children_.push_back(this);
      }
    }
    
    virtual ~Configurator()
    {
      for (ConfiguratorList::reverse_iterator ii=children_.rbegin(); ii != children_.rend(); ++ii)
        delete *ii;
    }

    Configurator *root()
    {
      Configurator *r = this;
      while (r && r->parent_)
        r = r->parent_;
      return r;
    }

    const Configurator *root() const
    {
      return const_cast<Configurator*>(this)->root();
    }
    
    const std::string &element() const
    {
      return element_;
    }
    
    std::string path() const
    {
      if (parent_)
        return parent_->path() + "/" + element_;
      else
        return element_;
    }

    ConfigurationParameter operator[](const std::string &path) const
    {
      const Configurator *c = find(path);
      if (c)
        return ConfigurationParameter(c->str());
      else
        throw Exception(Configurator::path() + ": Parameter '" + path + "' not set");
    }
    
    // *** virtual functions ***
    
    virtual std::string str() const
    {
      return std::string();
    }
    
    virtual Configurable *ptr()
    {
      return NULL;
    }

    // Find object in hierarchy.
    virtual Configurator *find(const std::string &path)
    {
      if (path.empty())
        return this;
        
      // Split path
      size_t pos = path.find_first_of('/');
      std::string first = path.substr(0, pos), rest;
      if (pos != std::string::npos)
        rest = path.substr(pos+1);

      if (first.empty())
      {
        return root()->find(rest);
      }
      else if (first == "..")
      {
        if (parent_)
          return parent_->find(rest);
        else
          return NULL;
      }
      else
      {
        for (ConfiguratorList::iterator ii=children_.begin(); ii != children_.end(); ++ii)
          if ((*ii)->element_ == first)
            return (*ii)->find(rest);

        return NULL;
      }
    }
    
    virtual const Configurator *find(const std::string &path) const
    {
      return const_cast<Configurator*>(this)->find(path);
    }
    
    virtual Configurator *instantiate(Configurator *parent=NULL) const
    {
      if (!parent)
        parent = parent_;
    
      Configurator *cfg = new Configurator(element_, parent);
    
      for (ConfiguratorList::const_iterator ii=children_.begin(); ii != children_.end(); ++ii)
        if (!(*ii)->instantiate(cfg))
          return NULL;
        
      return cfg;
    }
    
    virtual bool validate(const CRP &crp) const
    {
      return false;
    }

    virtual void reconfigure(const Configuration &config, bool recursive=false)
    {
      if (recursive)
        for (ConfiguratorList::iterator ii=children_.begin(); ii != children_.end(); ++ii)
          (*ii)->reconfigure(config, recursive);
    }
};

class ParameterConfigurator : public Configurator
{
  protected:
    std::string value_;
    
  public:
    ParameterConfigurator(const std::string &element, const std::string &value, Configurator *parent=NULL) : Configurator(element, parent), value_(value) { }
    std::string localize(const std::string &id) const;
    Configurator *resolve(const std::string &id);
    const Configurator *resolve(const std::string &id) const
    {
      return const_cast<ParameterConfigurator*>(this)->resolve(id);
    }
    
    virtual std::string str() const;
    virtual Configurable *ptr();
    virtual Configurator *find(const std::string &path);
    virtual const Configurator *find(const std::string &path) const
    {
      return const_cast<ParameterConfigurator*>(this)->find(path);
    }
    virtual ParameterConfigurator *instantiate(Configurator *parent=NULL) const;
    virtual bool validate(const CRP &crp) const;
    virtual void reconfigure(const Configuration &config, bool recursive=false);
};

class ObjectConfigurator : public Configurator
{
  protected:
    std::string type_;
    Configurable *object_;

  public:
    ObjectConfigurator(const std::string &element, const std::string &type, Configurator *parent=NULL) : Configurator(element, parent), type_(type), object_(NULL) { }
    virtual ~ObjectConfigurator();
    
    void attach(Configurable *object)
    {
      object_ = object;
    }
    
    void detach()
    {
      object_ = NULL;
    }

    virtual std::string str() const
    {
      std::ostringstream oss;
      oss << object_;
    
      return oss.str();
    }
    
    virtual Configurable *ptr()
    {
      return object_;
    }
    
    virtual ObjectConfigurator *instantiate(Configurator *parent=NULL) const;
    virtual bool validate(const CRP &crp) const;
    virtual void reconfigure(const Configuration &config, bool recursive=false);
};

Configurator *loadYAML(const std::string &file, const std::string &element=std::string(), Configurator *parent=NULL);
Configurator *loadYAML(const std::string &file, const std::string &element, Configurator *parent, const YAML::Node &node);

/// Configurable object.
class Configurable
{
  private:
    ObjectConfigurator *configurator_; ///< Configurator that instantiated this object.
    
  public:
    TYPEINFO("", "Base object")
    
    Configurable() : configurator_(NULL) { }

    virtual ~Configurable()
    {
      // Avoid double free by ObjectConfigurator in turn.
      if (configurator_)
      {
        CRAWL("Deleting associated configurator " << configurator_);
        
        configurator_->detach();
        safe_delete(&configurator_);
      }
    }
    
    void attach(ObjectConfigurator *configurator)
    {
      configurator_ = configurator;
    }
    
    void detach()
    {
      configurator_ = NULL;
    }
    
    /// Requested configurable parameters.
    virtual void request(ConfigurationRequest * /*config*/) { }
    
    /// Requested configurable parameters based on a specific role.
    virtual void request(const std::string &role, ConfigurationRequest *config) { request(config); }
    
    /**
     * \brief Configure the object.
     *
     * Only called once all parameters of the object have already been set and configured.
     */ 
    virtual void configure(Configuration &/*config*/) { }
    
    /// On-line reconfiguration and general messaging.
    virtual void reconfigure(const Configuration &/*config*/) { }
    
    /// Retrieve the path of this object definition in the configuration tree.
    std::string path() const
    {
      if (configurator_)
        return configurator_->path();
      else
        return std::string("<unknown>");
    }
    
    /// Reconfigure configuration subtree.
    void walk(const Configuration &config)
    {
      if (configurator_)
        configurator_->reconfigure(config, true);
    }
    
    /// Reset configuration subtree.
    void reset()
    {
      Configuration config;
      config.set("action", "reset");
      walk(config);
    }

  protected:
    /// Configurable-specific log writer (also logs class name).
    inline void log(unsigned char level, const std::ostringstream &oss) const
    {
      if (level <= grl_log_verbosity__)
      {
        if (level < 2)
          std::cerr << grl_log_levels__[level] << " " << path() << ": " << oss.str() << "\x1B[0m" << std::endl;
        else
          std::cout << grl_log_levels__[level] << " " << path() << ": " << oss.str() << "\x1B[0m" << std::endl;
      }
    }
};

DECLARE_FACTORY(Configurable)
#define REGISTER_CONFIGURABLE(subx) REGISTER_FACTORY(Configurable, subx, subx::s_type())

}

#endif /* GRL_CONFIGURABLE_H_ */
