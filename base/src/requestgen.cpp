/** \file requestgen.cpp
 * \brief Generates requests.yaml from object requests.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-25
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

#include <grl/grl.h>
#include <grl/configurable.h>
#include <grl/experiment.h>

using namespace grl;
using namespace std;

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    cerr << "Usage:" << endl
         << "  " << argv[0] << " <yaml file>" << endl;
    return 1;
  }
  
  // Load plugins
  loadPlugins();
  
  ConfigurableFactory::Map factories = ConfigurableFactory::factories();
  YAML::Node node;
  
  // Get requested parameters for all object factories
  for (ConfigurableFactory::Map::iterator ii=factories.begin(); ii != factories.end(); ++ii)
  {
    YAML::Node type;
  
    ConfigurationRequest request;
    
    Configurable *obj = ConfigurableFactory::create(ii->first);
    obj->request(&request);
    
    for (ConfigurationRequest::iterator jj=request.begin(); jj != request.end(); ++jj)
    {
      YAML::Node spec;
      spec["type"] = jj->type;
      spec["description"] = jj->description;
      switch (jj->mutability)
      {
        case CRP::Provided:
          spec["mutability"] = "provided";
          break;
        case CRP::System:
          spec["mutability"] = "system";
          break;
        case CRP::Configuration:
          spec["mutability"] = "configuration";
          break;
        case CRP::Online:
          spec["mutability"] = "online";
          break;
      }
      
      if (jj->mutability != CRP::Provided)
      {
        spec["default"] = jj->value;
        spec["optional"] = (int)jj->optional;
        
        if (jj->type == "int" || jj->type == "double")
        {
          spec["min"] = jj->min;
          spec["max"] = jj->max;
        }
        else if (jj->type == "string" && !jj->options.empty())
        {
          YAML::Node options;
          for (size_t kk=0; kk < jj->options.size(); ++kk)
            options.push_back(jj->options[kk]);
          spec["options"] = options;
        }
      }
      
      type[jj->name] = spec;
    }
    
    node[ii->first] = type;
    
    //delete obj;
  }
  
  std::ofstream ofs(argv[1]);
  
  ofs << YAML::Dump(node);
  
  return 0;
}
