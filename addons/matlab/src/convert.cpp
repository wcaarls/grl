/** \file convert.cpp
 * \brief Conversions between Matlab and grl datatypes source file.
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

#include <mex.h>
#include <grl/matlab/memstring.h>
#include <grl/matlab/convert.h>

using namespace grl;

mxArray *vectorToArray(const Vector &v)
{
  mxArray *pm = mxCreateDoubleMatrix(1, v.size(), mxREAL);
  double *dbl = mxGetPr(pm);
  for (size_t ii=0; ii < v.size(); ++ii)
    dbl[ii] = v[ii];
    
  return pm;
} 

Vector arrayToVector(const mxArray *pm)
{
  int sz;
  double *dbl;

  sz = mxGetNumberOfElements(pm);
  dbl = mxGetPr(pm);

  Vector v(sz);        
  for (int ii=0; ii < sz; ++ii)
    v[ii] = dbl[ii];

  return v;
}

void structToConfig(const mxArray *pm, Configuration &config)
{
  if (!mxIsStruct(pm))
    mexErrMsgTxt("Invalid configuration.");

  int fields = mxGetNumberOfFields(pm);
  
  for (int ii=0; ii < fields; ++ii)
  {
    const char *key = mxGetFieldNameByNumber(pm, ii);
    mxArray *value = mxGetFieldByNumber(pm, 0, ii);
    
    MexMemString str;
    double *dbl;
    int sz;
    
    switch (mxGetClassID(value))
    {
      case mxCHAR_CLASS:
        str = mxArrayToString(value);
        config.set(key, (char *)str);
        break;
      case mxDOUBLE_CLASS:
        sz = mxGetNumberOfElements(value);
        dbl = mxGetPr(value);
        
        if (sz == 1)
          config.set(key, *dbl);
        else
          config.set(key, arrayToVector(value));
          
        break;
      default:
        mexErrMsgTxt("Invalid data type in configuration.");
    }
  }
}

mxArray *taskSpecToStruct(const Configurator &config)
{
  const char *fieldnames[] = {"observation_dims", "observation_min", "observation_max",
                              "action_dims", "action_min", "action_max",
                              "reward_min", "reward_max"};

  std::string path;
  
  if (config.find("observation_min"))
    path = "";
  else if (config.find("task/observation_min"))
    path = "task/";
  else
    mexErrMsgTxt("Could not determine task specification.");

  mxArray *pm = mxCreateStructMatrix(1, 1, 8, fieldnames);
  
  mxSetField(pm, 0, "observation_dims", mxCreateDoubleScalar(config[path+"observation_dims"]));
  mxSetField(pm, 0, "observation_min", vectorToArray(config[path+"observation_min"]));
  mxSetField(pm, 0, "observation_max", vectorToArray(config[path+"observation_max"]));
  mxSetField(pm, 0, "action_dims", mxCreateDoubleScalar(config[path+"action_dims"]));
  mxSetField(pm, 0, "action_min", vectorToArray(config[path+"action_min"]));
  mxSetField(pm, 0, "action_max", vectorToArray(config[path+"action_max"]));
  mxSetField(pm, 0, "reward_min", mxCreateDoubleScalar(config[path+"reward_min"]));
  mxSetField(pm, 0, "reward_max", mxCreateDoubleScalar(config[path+"reward_max"]));
  
  return pm;
}
