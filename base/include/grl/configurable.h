#ifndef GRL_CONFIGURABLE_H_
#define GRL_CONFIGURABLE_H_

#include <yaml-cpp/yaml.h>

#include <grl/utils.h>
#include <grl/configuration.h>
#include <grl/factory.h>

namespace grl {

class ConfigurationRequestParameter
{
  std::string name, type;
  bool required;
  double min, max;
  std::vector<std::string> options;
  
  ConfigurationRequestParameter(std::string _name, std::string _type="",
    bool _required=false, double _min=0, double _max=0, std::vector<std::string> _options=std::vector<std::string>()) :
    name(_name), type(_type), required(_required), min(_min), max(_max), options(_options)
  {
    if (min == max)
    {
      min = -std::numeric_limits<double>::infinity();
      max =  std::numeric_limits<double>::infinity();
    }
  }
};

class ConfigurationRequest
{
};

#define TYPEINFO(t)\
    static std::string s_type() { return t; }\
    virtual std::string d_type() { return t; }

class Configurable
{
  public:
    virtual ~Configurable() { }
    
    TYPEINFO("")

    virtual void request(ConfigurationRequest */*config*/) { }
    virtual void configure(Configuration &/*config*/) { }
    virtual void reconfigure(const Configuration &/*config*/) { }
};

DECLARE_FACTORY(Configurable)
#define REGISTER_CONFIGURABLE(subx) REGISTER_FACTORY(Configurable, subx, subx::s_type())

class YAMLConfigurator
{
  protected:
    Configuration references_;

  public:
    void populate(const Configuration &config)
    {
      references_ = config;
    }
  
    Configurable *load(std::string file, Configuration *config)
    {
      return load(YAML::LoadFile(file.c_str()), config, "");
    }
  
    Configurable *load(const YAML::Node &node, Configuration *config, const std::string &path)
    {
      Configurable *obj=NULL;
      
      if (!node.IsMap())
        throw Exception("Can only load YAML maps");
        
      if (node["type"])
      {
        obj = ConfigurableFactory::create(node["type"].as<std::string>());
        
        if (!obj)
        {
          std::cerr << "Requested unknown object type " << node["type"] << std::endl;
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
        }
      }
      
      // TODO: Check config against request
      
      if (obj)
        obj->configure(*config);
        
      for (Configuration::MapType::const_iterator ii=config->parameters().begin(); ii != config->parameters().end(); ++ii)
        references_.set(path + ii->first, ii->second->str());
        
      return obj;
    }      
    
  protected:
    std::string toString(const YAML::Node &node)
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
};

}

#endif /* GRL_CONFIGURABLE_H_ */
