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

#include <unistd.h>
#include <sys/time.h>

#include <grl/grl.h>
#include <grl/configurable.h>
#include <grl/experiment.h>

using namespace grl;
using namespace std;

int main(int argc, char **argv)
{
  iSIGSEGV();

  int seed = 0;
  bool user_config = false;

  int c;
  while ((c = getopt (argc, argv, "vs:c")) != -1)
  {
    switch (c)
    {
      case 'v':
        grl_log_verbosity__++;
        break;
      case 's':
        seed = atoi(optarg);
        break;
      case 'c':
        user_config = true;
        break;
      default:
        return 1;    
    }
  }

  if (optind > argc-1)
  {
    ERROR("Usage: " << endl << "  " << argv[0] << " [-v] [-c] [-s seed] <yaml file> [yaml file...]");
    return 1;
  }
  
  if (seed)
  {
    srand(seed);
    srand48(seed);
  }
  else
  {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long unsigned int tseed = tv.tv_sec ^ tv.tv_usec ^ getpid();
  
    srand(tseed);
    srand48(tseed);
  }
  
  // Load plugins
  loadPlugins();
  
  Configurator *temp=NULL;
  for (; optind < argc; ++optind)
  {
    NOTICE("Loading configuration from '" << argv[optind] << "'");
    temp = loadYAML(argv[optind], "", temp);
  }
  
  if (!temp)
  {
    ERROR("Could not load configuration");
    return 1;
  }
  
  NOTICE("Instantiating configuration");
  configurator_ = temp->instantiate();
  delete temp;
  
  if (!configurator_)
  {
    ERROR("Could not instantiate configuration");
    return 1;
  }

  Configurator *expconf = configurator_->find("experiment");
  
  if (!expconf)
  {
    ERROR("YAML configuration does not specify an experiment");
    return 1;
  }
  
  Configurable *obj = expconf->ptr();
  Experiment *experiment = dynamic_cast<Experiment*>(obj);
  
  if (!experiment)
  {
    ERROR("Specified experiment has wrong type");
    return 1;
  }
  
  if (user_config)
    reconfigure();

  iSIGINT();

  NOTICE("Starting experiment");

  try
  {
    experiment->run();
  }
  catch (Exception &e)
  {
    ERROR(e.what());
    ERROR("Stack trace:\n" << e.trace());
  }
  
  NOTICE("Cleaning up");
  
  delete configurator_;
  
  NOTICE("Exiting");

  return 0;
}
