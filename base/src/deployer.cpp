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
#include <signal.h>
#include <string.h>

#include <grl/grl.h>
#include <grl/configurable.h>
#include <grl/experiment.h>

using namespace grl;
using namespace std;

static YAMLConfigurator *configurator = new YAMLConfigurator();
static struct sigaction act, oldact;

void reconfigure()
{
  NOTICE("Awaiting reconfiguration. Use [action] [object] [parameter=value]...");

  while (1)
  {
    Configuration config;
    std::string line;
    std::vector<std::string> words;
    getline(cin, line);
    if (line.empty())
      break;
      
    size_t seppos = 0, newpos;
    do
    {
      // Split into commands
      newpos = line.find(' ', seppos);
      std::string command = line.substr(seppos, newpos-seppos);
      
      if (!command.empty())
      {
        size_t eqpos = command.find('=');
        if (eqpos != std::string::npos)
        {
          // key=value
          std::string key = command.substr(0, eqpos);
          std::string value = command.substr(eqpos+1);
          
          config.set(key, value);
        }
        else
        {
          // action or object
          words.push_back(command);
        }
      }
      seppos = newpos+1;
    }
    while (newpos != std::string::npos);
    
    if (words.empty())
    {
      // <key=value...>
      configurator->reconfigure(config);
    }
    else if (words.size() == 1)
    {
      if (configurator->references().has(words[0]))
      {
        // object <key=value...>
        configurator->reconfigure(config, words[0]);
      }
      else
      {
        // action <key=value...>
        config.set("action", words[0]);
        configurator->reconfigure(config);
      }
    }
    else
    {
      // action object <key=value...>
      config.set("action", words[0]);
      configurator->reconfigure(config, words[1]);
    }
  }
  
  NOTICE("Reconfiguration complete");
}

void sighandler(int signum)
{
  sigaction(SIGINT, &oldact, NULL);

  reconfigure();
  
  sigaction(SIGINT, &act, NULL);
}

int main(int argc, char **argv)
{
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

  if (optind != argc-1)
  {
    ERROR("Usage: " << endl << "  " << argv[0] << " [-v] [-c] [-s seed] [-r] [-w] [-f file] <yaml file>");
    return 1;
  }
  
  if (seed)
  {
    srand(seed);
    srand48(seed);
  }
  else
  {
    srand(time(NULL));
    srand48(time(NULL));
  }
  
  // Load plugins
  loadPlugins();
  
  Configuration config;
  
  configurator->load(argv[optind], &config);
  
  Configurable *obj = (Configurable*)config["experiment"].ptr();
  Experiment *experiment = dynamic_cast<Experiment*>(obj);
  
  if (!experiment)
  {
    ERROR("Specified experiment has wrong type");
    return 1;
  }
    
  if (user_config)
    reconfigure();
  
  bzero(&act, sizeof(struct sigaction));
  act.sa_handler = sighandler;
  act.sa_flags = SA_NODEFER;
  sigaction(SIGINT, &act, &oldact);
  
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
  
  delete configurator;
  
  NOTICE("Exiting");

  return 0;
}
