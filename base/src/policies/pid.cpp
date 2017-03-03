/** \file pid.cpp
 * \brief PID policy source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-15
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

#include <grl/policies/pid.h>

#define P(i, o)  ((o)*setpoint_.size()+(i))
#define I(i, o)  (1*setpoint_.size()*outputs_+P(i, o))
#define D(i, o)  (2*setpoint_.size()*outputs_+P(i, o))
#define IL(i, o) (3*setpoint_.size()*outputs_+P(i, o))

using namespace grl;

REGISTER_CONFIGURABLE(PIDPolicy)
REGISTER_CONFIGURABLE(PIDTrajectoryPolicy)

void PIDPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("setpoint", "Setpoint", setpoint_, CRP::Online));
  config->push_back(CRP("outputs", "int.action_dims", "Number of outputs", (int)outputs_, CRP::System, 1));
  
  config->push_back(CRP("p", "P gains ([out1_in1, ..., out1_inN, ..., outN_in1, ..., outN_inN])", p_, CRP::Online));
  config->push_back(CRP("i", "I gains", i_, CRP::Online));
  config->push_back(CRP("d", "D gains (use P gain on velocity instead, if available)", d_, CRP::Online));
  config->push_back(CRP("il", "Integration limits", il_, CRP::Online));

  config->push_back(CRP("action_min", "vector.action_min", "Lower limit on actions", action_min_, CRP::System));
  config->push_back(CRP("action_max", "vector.action_max", "Upper limit on actions", action_max_, CRP::System));
}

void PIDPolicy::configure(Configuration &config)
{
  action_min_ = config["action_min"].v();
  action_max_ = config["action_max"].v();

  setpoint_ = config["setpoint"].v();
  if (!setpoint_.size())
    throw bad_param("policy/pid:setpoint");
    
  outputs_ = config["outputs"];

  if (action_min_.size() != action_max_.size() || action_max_.size() != outputs_)
    throw bad_param("policy/pid:{action_min,action_max,outputs}");

  p_ = config["p"].v();
  if (!p_.size())
    p_ = ConstantLargeVector(setpoint_.size()*outputs_, 0.);
  if (p_.size() != setpoint_.size()*outputs_)
    throw bad_param("policy/pid:p");

  i_ = config["i"].v();
  if (!i_.size())
    i_ = ConstantLargeVector(setpoint_.size()*outputs_, 0.);
  if (i_.size() != setpoint_.size()*outputs_)
    throw bad_param("policy/pid:i");

  d_ = config["d"].v();
  if (!d_.size())
    d_ = ConstantLargeVector(setpoint_.size()*outputs_, 0.);
  if (d_.size() != setpoint_.size()*outputs_)
    throw bad_param("policy/pid:d");

  il_ = config["il"].v();
  if (!il_.size())
    il_ = ConstantLargeVector(setpoint_.size()*outputs_, std::numeric_limits<double>::infinity());
  if (il_.size() != setpoint_.size()*outputs_)
    throw bad_param("policy/pid:il");

  params_ = extend(extend(extend(p_, i_), d_), il_);

  reset();
}

void PIDPolicy::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    ival_ = ConstantLargeVector(setpoint_.size()*outputs_, 0.);
  }
}

void PIDPolicy::act(const Observation &in, Action *out) const
{
  out->v.resize(outputs_);
  out->type = atGreedy;

  for (size_t oo=0; oo < outputs_; ++oo)
  {
    double u = 0;
    
    for (size_t ii=0; ii < setpoint_.size(); ++ii)
    {
      double err = setpoint_[ii] - in[ii];
      
      // Autonomous policy assumes no accumulated errors or differences, but
      // integration happens before applying the gains.
      u += (params_[P(ii, oo)]+params_[I(ii, oo)])*err;
    }

    (*out)[oo] = fmin(action_max_[oo], fmax(u, action_min_[oo]));
  }
}

void PIDPolicy::act(double time, const Observation &in, Action *out)
{
  if (time == 0.)
  {
    // First action in episode, clear integrator
    ival_ = ConstantLargeVector(setpoint_.size()*outputs_, 0.);
    prev_in_ = in;
  }

  out->v.resize(outputs_); 
  out->type = atGreedy;

  for (size_t oo=0; oo < outputs_; ++oo)
  {
    double u = 0;
    
    for (size_t ii=0; ii < setpoint_.size(); ++ii)
    {
      double err = setpoint_[ii] - in[ii];
      double acc = fmin(ival_[ii*outputs_+oo] + err, params_[IL(ii, oo)]);
      double diff = in[ii] - prev_in_[ii];
      
      u += params_[P(ii, oo)]*err + params_[I(ii, oo)]*acc + params_[D(ii, oo)]*diff;
      
      ival_[ii*outputs_+oo] = acc;
    }
    
    (*out)[oo] = fmin(action_max_[oo], fmax(u, action_min_[oo]));
  }
  
  prev_in_ = in;
}

////////////////////////////////////////////////
void PIDTrajectoryPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("trajectory", "mapping", "Maps time to setpoints", trajectory_));

  config->push_back(CRP("inputs", "int.observation_dims", "Number of inputs", (int)inputs_, CRP::System, 1));
  config->push_back(CRP("outputs", "int.action_dims", "Number of outputs", (int)outputs_, CRP::System, 1));

  config->push_back(CRP("p", "P gains ([out1_in1, ..., out1_inN, ..., outN_in1, ..., outN_inN])", p_, CRP::Online));
  config->push_back(CRP("i", "I gains", i_, CRP::Online));
  config->push_back(CRP("d", "D gains (use P gain on velocity instead, if available)", d_, CRP::Online));
  config->push_back(CRP("il", "Integration limits", il_, CRP::Online));

  config->push_back(CRP("action_min", "vector.action_min", "Lower limit on actions", action_min_, CRP::System));
  config->push_back(CRP("action_max", "vector.action_max", "Upper limit on actions", action_max_, CRP::System));
}

void PIDTrajectoryPolicy::configure(Configuration &config)
{
  action_min_ = config["action_min"].v();
  action_max_ = config["action_max"].v();

  trajectory_ = (Mapping*)config["trajectory"].ptr();

  inputs_ = config["inputs"];
  outputs_ = config["outputs"];

  if (action_min_.size() != action_max_.size() || action_max_.size() != outputs_)
    throw bad_param("policy/pid:{action_min,action_max,outputs}");

  p_ = config["p"].v();
  if (!p_.size())
    p_ = ConstantLargeVector(inputs_*outputs_, 0.);
  if (p_.size() && p_.size() != inputs_*outputs_)
    throw bad_param("policy/pidt:p");

  i_ = config["i"].v();
  if (!i_.size())
    i_ = ConstantLargeVector(inputs_*outputs_, 0.);
  if (i_.size() && i_.size() != inputs_*outputs_)
    throw bad_param("policy/pidt:i");

  d_ = config["d"].v();
  if (!d_.size())
    d_ = ConstantLargeVector(inputs_*outputs_, 0.);
  if (d_.size() && d_.size() != inputs_*outputs_)
    throw bad_param("policy/pidt:d");

  il_ = config["il"].v();
  if (!il_.size())
    il_ = ConstantLargeVector(inputs_*outputs_, std::numeric_limits<double>::infinity());
  if (il_.size() && il_.size() != inputs_*outputs_)
    throw bad_param("policy/pidt:il");

  params_ = extend(extend(extend(p_, i_), d_), il_);

  reset();
}

void PIDTrajectoryPolicy::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    ival_ = ConstantLargeVector(inputs_*outputs_, 0.);
  }
}

void PIDTrajectoryPolicy::act(double time, const Observation &in, Action *out)
{
  // Read current setpoint from the trajectory
  trajectory_->read(VectorConstructor(time), &setpoint_);
  
  if (setpoint_.size() != inputs_)
    throw bad_param("policy/pidt:inputs");

  if (time == 0.)
  {
    // First action in episode, clear integrator
    ival_ = ConstantLargeVector(inputs_*outputs_, 0.);
    prev_in_ = in;
  }

  out->v.resize(outputs_);
  out->type = atGreedy;

  for (size_t oo=0; oo < outputs_; ++oo)
  {
    double u = 0;

    for (size_t ii=0; ii < inputs_; ++ii)
    {
      double err = setpoint_[ii] - in[ii];
      double acc = fmin(ival_[ii*outputs_+oo] + err, params_[IL(ii, oo)]);
      double diff = in[ii] - prev_in_[ii];

      u += params_[P(ii, oo)]*err + params_[I(ii, oo)]*acc + params_[D(ii, oo)]*diff;
      
      ival_[ii*outputs_+oo] = acc;
    }

    (*out)[oo] = fmin(action_max_[oo], fmax(u, action_min_[oo]));
  }

  prev_in_ = in;
}
