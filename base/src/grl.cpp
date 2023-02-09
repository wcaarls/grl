/** \file grl.cpp
 * \brief Main GRL sourcefile.
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
#include <libgen.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

#include <grl/configurable.h>
#include <grl/projections/sample.h>
#include <grl/visualization.h>

using namespace grl;

DEFINE_FACTORY(Configurable)

DECLARE_TYPE_NAME(bool)
DECLARE_TYPE_NAME(char)
DECLARE_TYPE_NAME(unsigned char)
DECLARE_TYPE_NAME(short int)
DECLARE_TYPE_NAME(short unsigned int)
DECLARE_TYPE_NAME(int)
DECLARE_TYPE_NAME(unsigned int)
DECLARE_TYPE_NAME(long int)
DECLARE_TYPE_NAME(long unsigned int)
DECLARE_TYPE_NAME(long long int)
DECLARE_TYPE_NAME(long long unsigned int)
DECLARE_TYPE_NAME(float)
DECLARE_TYPE_NAME(double)
DECLARE_TYPE_NAME(Vector)
#ifndef GRL_VECTOR_IS_LARGE_VECTOR
DECLARE_TYPE_NAME(LargeVector)
#endif
DECLARE_TYPE_NAME(std::string)

ReadWriteLock SampleStore::rwlock_;

Visualizer *Visualizer::instance_ = NULL;

pthread_once_t RandGen::once_ = PTHREAD_ONCE_INIT;
pthread_mutex_t RandGen::mutex_;
pthread_key_t RandGen::key_;

Configurator *grl::configurator_ = NULL;
struct sigaction grl::act_, grl::oldact_;

static void loadPlugins1(const std::string &pattern)
{
  glob_t globbuf;
  
  glob(pattern.c_str(), 0, NULL, &globbuf);
  for (size_t ii=0; ii < globbuf.gl_pathc; ++ii)
  { 
    NOTICE("Loading plugin '" << globbuf.gl_pathv[ii] << "'");
    if (!dlopen(globbuf.gl_pathv[ii], RTLD_NOW|RTLD_GLOBAL))
      ERROR("Error loading plugin '" << globbuf.gl_pathv[ii] << "': " << dlerror());
  } 
}

std::string grl::getLibraryPath()
{
  Dl_info dl_info;
  
  // ISO C++ forbids casting between pointer-to-function and pointer-to-object
  #ifdef __GNUC__
  __extension__
  #endif
  dladdr((void *)grl::getLibraryPath, &dl_info);

  char buf[PATH_MAX] = { 0 };
  strcpy(buf, dl_info.dli_fname);

  std::string path = dirname(buf);
  
  return path;
}

void grl::loadPlugins()                   
{
  std::string pattern = getLibraryPath() + "/libaddon*.so";
  loadPlugins1(pattern);

  pattern = getLibraryPath() + "/grl/libaddon*.so";
  loadPlugins1(pattern);
}

void grl::reconfigure()
{
  NOTICE("Awaiting reconfiguration. Use [action] [object] [parameter=value]...");

  while (1)
  {
    Configuration config;
    std::string line;
    std::vector<std::string> words;
    bool set=false;
    
    getline(std::cin, line);
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
          set=true;
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
      // [key=value...]
      configurator_->reconfigure(config, true);

      if (config.has("verbose"))
        grl_log_verbosity__ = config["verbose"].i();
    }
    else if (words.size() == 1)
    {
      Configurator *cfg = configurator_->find(words[0]);
      if (cfg)
      {
        ObjectConfigurator *oc = dynamic_cast<ObjectConfigurator*>(cfg);
        
        if (oc)
        {
          if (set)
          {
            // object <key=value...>
            oc->reconfigure(config);
          }
          else
          {
            // object
            Configurable *obj = dynamic_cast<Configurable*>(oc->ptr());
            if (obj)
            {
              ConfigurationRequest request;
              obj->request(&request);
            
              std::cout << oc->path() << ":" << std::endl;
              for (size_t ii=0; ii != request.size(); ++ii)
                if (request[ii].mutability != CRP::Provided)
                  std::cout << "  " << request[ii].name << ": " << request[ii].value << std::endl;
            }
            else
              WARNING("Cannot inspect '" << words[0] << "': not instantiated");
          }
        }
        else
        {
          WARNING("Cannot reconfigure '" << words[0] << "': not a configurable object.");
        }
      }
      else
      {
        // action [key=value...]
        config.set("action", words[0]);
        configurator_->reconfigure(config, true);
      }
    }
    else
    {
      // action object <key=value...>
      config.set("action", words[0]);
      
      Configurator *cfg = configurator_->find(words[1]);
      if (cfg)
      {
        ObjectConfigurator *oc = dynamic_cast<ObjectConfigurator*>(cfg);
        
        if (oc)
          oc->reconfigure(config);
        else
          WARNING("Cannot reconfigure '" << words[1] << "': not a configurable object.");
      }
      else
        WARNING("Cannot reconfigure '" << words[1] << "': no such object.");
    }
  }
  
  NOTICE("Reconfiguration complete");
}

void grl::iSIGINT()
{
  bzero(&act_, sizeof(struct sigaction));
  act_.sa_handler = hSIGINT;
  act_.sa_flags = SA_NODEFER;
  sigaction(SIGINT, &act_, &oldact_);
}

void grl::iSIGSEGV()
{
  signal(SIGSEGV, hSIGSEGV);
}

void grl::hSIGINT(int sig)
{
  sigaction(SIGINT, &oldact_, NULL);

  reconfigure();
  
  sigaction(SIGINT, &act_, NULL);
}

void grl::hSIGSEGV(int sig)
{
  ERROR("Caught SIGSEGV");
  ERROR("Stack trace:\n" << stacktrace());
  exit(1);
}
