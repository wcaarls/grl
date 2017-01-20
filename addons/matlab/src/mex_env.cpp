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

#include <string.h>
#include <mex.h>

#include <grl/grl.h>

#include <grl/matlab/memstring.h>
#include <grl/matlab/convert.h>

#include <grl/configuration.h>
#include <grl/environment.h>

using namespace grl;

static Configurator *g_configurator=NULL;
static Environment *g_env=NULL;
static int g_action_dims=0;
static bool g_started=false;

void mexFunction(int nlhs, mxArray *plhs[ ],
                 int nrhs, const mxArray *prhs[ ])
{
  MexMemString func;
  static bool first=true;
  
  if (first)
  {
    loadPlugins();
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
      
    Configurator *conf, *envconf;
    if (!(conf = loadYAML(file)) || !(g_configurator = conf->instantiate()) || !(envconf = g_configurator->find("environment")))
    {
      safe_delete(&conf);
      safe_delete(&g_configurator);
      return;
    }
    
    safe_delete(&conf);
    g_env = dynamic_cast<Environment*>(envconf->ptr());
    
    if (!g_env)
    {
      safe_delete(&g_configurator);
      mexErrMsgTxt("Configuration file does not specify a valid environment.");
    }

    plhs[0] = taskSpecToStruct(*envconf);
    
    g_action_dims = mxGetPr(mxGetField(plhs[0], 0, "action_dims"))[0];
    g_started = false;

    mexLock();

    return;
  }
  
  if (!g_env)
    mexErrMsgTxt("Not initialized.");

  if (!strcmp(func, "fini"))
  {
    safe_delete(&g_configurator);
    g_env = NULL;
    mexUnlock();
  }
  else if (!strcmp(func, "start"))
  {
    // Run environment
    Observation obs;
    
    // TODO: READ TEST ARGUMENT
    g_env->start(0, &obs);
    g_started = true;
    
    // Process output
    plhs[0] = vectorToArray(obs);
  }
  else if (!strcmp(func, "step"))
  {
    Action action;
    
    if (!g_started)
      mexErrMsgTxt("Environment not started.");

    // Verify input    
    if (nrhs < 2 || !mxIsDouble(prhs[1]))
      mexErrMsgTxt("Missing action.");
      
    // Prepare input
    int elements = mxGetNumberOfElements(prhs[1]);
    
    if (elements != g_action_dims)
      mexErrMsgTxt("Invalid action size.");
    
    action.v.resize(elements);
    for (size_t ii=0; ii < elements; ++ii)
      action[ii] = mxGetPr(prhs[1])[ii];
    
    // Run environment
    Observation obs;
    double reward;
    int terminal;
    double tau = g_env->step(action, &obs, &reward, &terminal);
    
    // Process output
    plhs[0] = vectorToArray(obs);
    if (nlhs > 1) 
      plhs[1] = mxCreateDoubleScalar(reward);
    if (nlhs > 2)
      plhs[2] = mxCreateDoubleScalar(terminal);
    if (nlhs > 3)
      plhs[3] = mxCreateDoubleScalar(tau);
  }
  else
    mexErrMsgTxt("Unknown command.");
}
