/** \file mex_env.cpp
 * \brief Matlab access to grl environments.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-17
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

#include <string.h>
#include <mex.h>

#include <grl/matlab/memstring.h>
#include <grl/matlab/convert.h>

#include <grl/configuration.h>
#include <grl/environment.h>

using namespace grl;

static Environment *g_env=NULL;

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

void mexFunction(int nlhs, mxArray *plhs[ ],
                 int nrhs, const mxArray *prhs[ ])
{
  MexMemString func;
  static bool first=true;
  
  if (first)
  {
    loadPlugins("libaddon*.so");
    first = false;
  }

  if (nrhs < 1 || !mxIsChar(prhs[0]) || !(func = mxArrayToString(prhs[0])))
    mexErrMsgTxt("Missing function name.");

  if (!strcmp(func, "init"))
  {
    MexMemString file;
  
    if (g_env)
      mexErrMsgTxt("Already initialized.");
      
    if (nrhs < 2 || !mxIsChar(prhs[1]) || !(file = mxArrayToString(prhs[1])))
      mexErrMsgTxt("Missing configuration file name.");
      
    Configuration config;
    YAMLConfigurator configurator;
    
    configurator.load(file, &config);
  
    Configurable *obj = (Configurable*)config["environment"].ptr();
    g_env = dynamic_cast<Environment*>(obj);
    
    if (!g_env)
      mexErrMsgTxt("Configuration file does not specify a valid environment.");

    plhs[0] = taskSpecToStruct(configurator.references());

    mexLock();

    return;
  }
  
  if (!g_env)
    mexErrMsgTxt("Not initialized.");

  if (!strcmp(func, "fini"))
  {
    delete g_env;
    g_env = NULL;
    mexUnlock();
  }
  else if (!strcmp(func, "start"))
  {
    // Run environment
    Vector obs;
    g_env->start(&obs);
    
    // Process output
    plhs[0] = vectorToArray(obs);
  }
  else if (!strcmp(func, "step"))
  {
    Vector action;

    // Verify input    
    if (nrhs < 2 || !mxIsDouble(prhs[1]))
      mexErrMsgTxt("Missing action.");
    
    // Prepare input
    int elements = mxGetNumberOfElements(prhs[1]);
    action.resize(elements);
    for (size_t ii=0; ii < elements; ++ii)
      action[ii] = mxGetPr(prhs[1])[ii];
    
    // Run environment
    Vector obs;
    double reward;
    int terminal;
    g_env->step(action, &obs, &reward, &terminal);
    
    // Process output
    plhs[0] = vectorToArray(obs);
    if (nlhs > 1) 
      plhs[1] = mxCreateDoubleScalar(reward);
    if (nlhs > 2)
      plhs[2] = mxCreateDoubleScalar(terminal);
  }
  else
    mexErrMsgTxt("Unknown command.");
}
