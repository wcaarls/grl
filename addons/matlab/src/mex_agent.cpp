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

static Configurator *g_configurator=NULL;
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
      
    Configurator *conf, *agentconf;
    if (!(conf = loadYAML(file)) || !(g_configurator = conf->instantiate()) || !(agentconf = g_configurator->find("agent")))
    {
      safe_delete(&conf);
      safe_delete(&g_configurator);
      return;
    }
    
    safe_delete(&conf);
    g_agent = dynamic_cast<Agent*>(agentconf->ptr());
    
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
      mexErrMsgTxt("Missing tau.");
    
    if (nrhs < 3 || !mxIsDouble(prhs[2]))
      mexErrMsgTxt("Missing reward.");
    
    if (nrhs < 4 || !mxIsDouble(prhs[3]))
      mexErrMsgTxt("Missing state.");
      
    if (mxGetNumberOfElements(prhs[1]) != 1)
      mexErrMsgTxt("Invalid tau.");

    if (mxGetNumberOfElements(prhs[2]) != 1)
      mexErrMsgTxt("Invalid reward.");
    
    // Prepare input
    double tau = mxGetPr(prhs[1])[0];
    double reward = mxGetPr(prhs[2])[0];
    Vector obs = arrayToVector(prhs[3]);
  
    // Run agent
    g_agent->step(tau, obs, reward, &action);
    
    // Process output
    plhs[0] = vectorToArray(action);
  }
  else if (!strcmp(func, "end"))
  {
    // Verify input    
    if (nrhs < 2 || !mxIsDouble(prhs[1]))
      mexErrMsgTxt("Missing tau.");
    
    if (nrhs < 3 || !mxIsDouble(prhs[2]))
      mexErrMsgTxt("Missing reward.");
    
    if (nrhs < 4 || !mxIsDouble(prhs[3]))
      mexErrMsgTxt("Missing state.");
      
    if (mxGetNumberOfElements(prhs[1]) != 1)
      mexErrMsgTxt("Invalid tau.");

    if (mxGetNumberOfElements(prhs[2]) != 1)
      mexErrMsgTxt("Invalid reward.");
    
    // Prepare input
    double tau = mxGetPr(prhs[1])[0];
    double reward = mxGetPr(prhs[2])[0];
    Vector obs = arrayToVector(prhs[3]);
  
    // Run agent
    g_agent->end(tau, obs, reward);
  }
  else
    mexErrMsgTxt("Unknown command.");
}
