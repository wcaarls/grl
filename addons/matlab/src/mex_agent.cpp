/** \file mex_agent.cpp
 * \brief Matlab access to grl agents.
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
#include <grl/agent.h>

using namespace grl;

static YAMLConfigurator *g_configurator=NULL;
static Agent *g_agent=NULL;

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
  
    if (g_agent)
      mexErrMsgTxt("Already initialized.");
      
    if (nrhs < 2 || !mxIsChar(prhs[1]) || !(file = mxArrayToString(prhs[1])))
      mexErrMsgTxt("Missing configuration file name.");
      
    Configuration config;
    g_configurator = new YAMLConfigurator;
    g_configurator->load(file, &config);

    Configurable *obj=NULL;
    if (config.has("agent"))
      obj = (Configurable*)config["agent"].ptr();
    g_agent = dynamic_cast<Agent*>(obj);
    
    if (!g_agent)
    {
      safe_delete(&g_configurator);
      mexErrMsgTxt("Configuration file does not specify a valid agent.");
    }

    mexLock();

    return;
  }
  
  if (!g_agent)
    mexErrMsgTxt("Not initialized.");

  if (!strcmp(func, "fini"))
  {
    safe_delete(&g_configurator);
    g_agent = NULL;
    mexUnlock();
  }
  else if (!strcmp(func, "start"))
  {
    Vector action;
    
    // Verify input    
    if (nrhs < 2 || !mxIsDouble(prhs[1]))
      mexErrMsgTxt("Missing state.");
    
    // Prepare input
    Vector obs = arrayToVector(prhs[1]);
  
    // Run agent
    g_agent->start(obs, &action);
    
    // Process output
    plhs[0] = vectorToArray(action);
  }
  else if (!strcmp(func, "step"))
  {
    Vector action;
    
    // Verify input    
    if (nrhs < 2 || !mxIsDouble(prhs[1]))
      mexErrMsgTxt("Missing reward.");
    
    if (nrhs < 3 || !mxIsDouble(prhs[2]))
      mexErrMsgTxt("Missing state.");
    
    if (mxGetNumberOfElements(prhs[1]) != 1)
      mexErrMsgTxt("Invalid reward.");
    
    // Prepare input
    double reward = mxGetPr(prhs[1])[0];
    Vector obs = arrayToVector(prhs[2]);
  
    // Run agent
    g_agent->step(obs, reward, &action);
    
    // Process output
    plhs[0] = vectorToArray(action);
  }
  else if (!strcmp(func, "end"))
  {
    // Verify input    
    if (nrhs < 2 || !mxIsDouble(prhs[1]))
      mexErrMsgTxt("Missing reward.");
    
    if (mxGetNumberOfElements(prhs[1]) != 1)
      mexErrMsgTxt("Invalid reward.");
    
    // Prepare input
    double reward = mxGetPr(prhs[1])[0];
  
    // Run agent
    g_agent->end(reward);
  }
  else
    mexErrMsgTxt("Unknown command.");
}
