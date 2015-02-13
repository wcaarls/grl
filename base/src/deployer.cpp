/** \file deployer.cpp
 * \brief Deployer for object configurations specified through YAML.
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

#include <glob.h>
#include <dlfcn.h>

#include <grl/configurable.h>
#include <grl/experiment.h>

using namespace grl;
using namespace std;

void loadPlugins(const char *pattern)
{
  glob_t globbuf;
  
  glob(pattern, 0, NULL, &globbuf);
  for (size_t ii=0; ii < globbuf.gl_pathc; ++ii)
  { 
    NOTICE("Loading plugin '" << globbuf.gl_pathv[ii] << "'");
    if (!dlopen(globbuf.gl_pathv[ii], RTLD_NOW|RTLD_LOCAL))
      ERROR("Error loading plugin '" << globbuf.gl_pathv[ii] << "': " << dlerror());
  } 
}

int main(int argc, char **argv)
{
  int c;
  while ((c = getopt (argc, argv, "v")) != -1)
  {
    switch (c)
    {
      case 'v':
        grl_log_verbosity__++;
        break;
      default:
        return 1;    
    }
  }

  if (optind != argc-1)
  {
    ERROR("Usage: " << endl << "  " << argv[0] << " [options] <yaml file>");
    return 1;
  }
  
  // Load plugins
  loadPlugins("libaddon*.so");
  
  Configuration config, task_spec;
  YAMLConfigurator configurator;
  
  configurator.populate(task_spec);
  
  Configurable *obj = configurator.load(argv[optind], &config);
  Experiment *experiment = dynamic_cast<Experiment*>(obj);
  
  if (!experiment)
  {
    ERROR("Configuration root must specify an experiment");
    return 1;
  }
  
  experiment->run();
  
  return 0;
}
