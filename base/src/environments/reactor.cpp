/** \file reactor.cpp
 * \brief van de Vusse continuous stirred tank reactor environment source file.
 *
 * \author    Eric Luz <ercluz@gmail.com>
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2019-07-17
 *
 * \copyright \verbatim
 * Copyright (c) 2019, Wouter Caarls
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

#include <grl/environments/reactor.h>

using namespace grl;

REGISTER_CONFIGURABLE(ReactorDynamics)
REGISTER_CONFIGURABLE(ReactorBalancingTask)
REGISTER_CONFIGURABLE(ReactorTrackingTask)
REGISTER_CONFIGURABLE(ReactorMaximizationTask)

void ReactorDynamics::request(ConfigurationRequest *config)
{
}

void ReactorDynamics::configure(Configuration &config)
{
  k0_ = Vector(3);
  Ea_ = Vector(3);
  Dh_ = Vector(3);

  k0_   << 357500000, 357500000, 2511900; // [1/s, 1/s, L/(mol s)] Kinematic constants
  Ea_   << -9758.3, -9758.3, -8560.0; // [K]                   Activation energy
  Dh_   << 4.2, -11, -41.85;          // [kJ/mol]              Formation enthalpy
  ro_   = 0.9342;                     // [kg/L]                Density
  Cp_   = 3.01;                       // [kJ/kgK]              Heat capacity of the reactor
  v_    = 10;                         // [L]                   Volume of the reactor
  kw_   = 1.12;                       // [kJ/s K m^2]          Heat transfer coefficient
  Ar_   = 0.215;                      // [m^2]                 Heat exchange area
  mk_   = 5;                          // [kg]                  Mass of the jacket
  Cpk_  = 2.0;                        // [kJ/kgK]              Heat capacity of the jacket
  
  Cain_ = 5.1; // 3.3-5.5             // [mol/L]               Input concentration of A
  Cbin_ = 0.0;                        // [mol/L]               Input concentration of B
  Tin_  = 400; // ???-???             // [K]                   Temperature of reactor input
  Tkf_  = 298; // 303-333             // [K]                   Temperature of jacket input
  Vk_   = 5;                          // [L]                   Volume of the jacket
}

void ReactorDynamics::reconfigure(const Configuration &config)
{
}

void ReactorDynamics::eom(const Vector &state, const Vector &actuation, Vector *xd) const
{
  if (state.size() != 5 || actuation.size() != 2)
    throw Exception("dynamics/reactor requires a task/reactor subclass");
    
  double Ca = state[0], Cb = state[1], T = state[2], Tk = state[3];
  double Fin = actuation[0], Fink = actuation[1];

  // Residence time
  double tau = v_ / Fin;

  // A -> B
  double r1 = k0_[0]*exp(Ea_[0]/T)*Ca;
  
  // B -> C
  double r2 = k0_[1]*exp(Ea_[1]/T)*Cb;
  
  // 2A -> D
  double r3 = k0_[2]*exp(Ea_[2]/T)*Ca*Ca;
  
  // Balance of species A
  double Cad = (1/tau) * ( Cain_ - Ca ) - r1 - r3;
  
  // Balance of species B
  double Cbd = (1/tau) * ( Cbin_ - Cb ) + r1 - r2;

  // Thermal balance of the reactor
  double Td = (1/tau) * (Tin_-T) + ((kw_*Ar_)/(ro_*Cp_*v_))*(Tk-T)-(1/(ro_*Cp_))*(r1*Dh_[0]+r2*Dh_[1]+r3*Dh_[2]);
  
  // Thermal balance of the jacket
  double Tkd = (Fink/Vk_)*(Tkf_-Tk) + ((kw_*Ar_)/(mk_*Cpk_))*(T-Tk);
  
  xd->resize(5);
  (*xd)[0] = Cad;
  (*xd)[1] = Cbd;
  (*xd)[2] = Td;
  (*xd)[3] = Tkd;
  (*xd)[4] = 1;
}

/// ReactorTask
void ReactorTask::request(ConfigurationRequest *config)
{
  Task::request(config);

  config->push_back(CRP("timeout", "Episode timeout", T_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("randomization", "Level of start state randomization", randomization_, CRP::Configuration, 0., 1.));
}

void ReactorTask::configure(Configuration &config)
{
  T_ = config["timeout"];
  randomization_ = config["randomization"];

  config.set("observation_dims", 4);
  config.set("observation_min", VectorConstructor(3.3, 0.0, 285., 285.));
  config.set("observation_max", VectorConstructor(5.5, 1.3, 450., 450.));
  config.set("action_dims", 2);
  config.set("action_min", VectorConstructor(  0.,   0.));
  config.set("action_max", VectorConstructor(700., 400.));
}

void ReactorTask::reconfigure(const Configuration &config)
{
}

void ReactorTask::start(int test, Vector *state)
{
  state->resize(5);
  (*state)[0] = 5.1  + (test==0)*randomization_*0.4*(RandGen::get()*2-1);
  (*state)[1] = 0    + (test==0)*randomization_*1.0*(RandGen::get());
  (*state)[2] = 380  + (test==0)*randomization_*50.*(RandGen::get()*2-1);;
  (*state)[3] = 380  + (test==0)*randomization_*50.*(RandGen::get()*2-1);;
  (*state)[4] = 0;
}

bool ReactorTask::actuate(const Vector &prev, const Vector &state, const Action &action, Vector *actuation) const
{
  // Limit and convert feed rates from hours to seconds
  *actuation = VectorConstructor(fmin(fmax(action[0], 0), 700)/3600,
                                 fmin(fmax(action[1], 0), 400)/3600);
  return true;
}

void ReactorTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  if (state.size() != 5)
    throw Exception("task/reactor requires dynamics/reactor");

  obs->v.resize(4);
  (*obs)[0] = state[0];
  (*obs)[1] = state[1];
  (*obs)[2] = state[2];
  (*obs)[3] = state[3];

  obs->absorbing = false;
  
  if (state[4] > T_)
    *terminal = 1;
  else
    *terminal = 0;
}

bool ReactorTask::invert(const Observation &obs, Vector *state, double time) const
{
  state->resize(5);
  (*state)[0] = obs[0];
  (*state)[1] = obs[1];
  (*state)[2] = obs[2];
  (*state)[3] = obs[3];
  (*state)[4] = time;

  return true;
}

/// ReactorBalancingTask

void ReactorBalancingTask::request(ConfigurationRequest *config)
{
  ReactorTask::request(config);

  config->push_back(CRP("setpoint", "Fb setpoint", setpoint_, CRP::Configuration, 0., DBL_MAX));
}

void ReactorBalancingTask::configure(Configuration &config)
{
  ReactorTask::configure(config);

  setpoint_ = config["setpoint"];

  config.set("reward_min", -sqrt(570));
  config.set("reward_max", 1.3);
}

void ReactorBalancingTask::evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const
{
  if (state.size() != 5 || action.size() != 2 || next.size() != 5)
    throw Exception("task/reactor/balancing requires dynamics/reactor");

  // Fb in [mol/h]
  double Fb = action[0]*(state[1]+next[1])/2;

  // Maximize Cb while keeping Fb at setpoint  
  *reward = state[1] - 0.1*sqrt(abs(Fb - setpoint_));
  
  // Normalize reward per timestep.
  *reward *= (next[4]-state[4]);
}

/// ReactorTrackingTask

void ReactorTrackingTask::request(ConfigurationRequest *config)
{
  ReactorTask::request(config);

  config->push_back(CRP("mpc", "MPC mode: observe time and use quadratic rewards", mpc_, CRP::Configuration, 0, 1));
  config->push_back(CRP("min", "Minimum Fb setpoint", min_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("max", "Minimum Fb setpoint", max_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("setpoints", "Number of random setpoints for training", setpoints_, CRP::Configuration, 1));

  config->push_back(CRP("profile", "mapping", "Setpoint profile for testing", profile_));

  config->push_back(CRP("fin_weight", "Relative weight of Fin maximization", fin_weight_, CRP::Configuration, 0., DBL_MAX));
  
}

void ReactorTrackingTask::configure(Configuration &config)
{
  ReactorTask::configure(config);

  mpc_ = config["mpc"];
  min_ = config["min"];
  max_ = config["max"];
  setpoints_ = config["setpoints"];
  
  profile_ = (Mapping*)config["profile"].ptr();
  fin_weight_ = config["fin_weight"];

  config.set("observation_dims", 5+mpc_);
  config.set("observation_min", extend(config["observation_min"].v(), VectorConstructor(min_)));
  config.set("observation_max", extend(config["observation_max"].v(), VectorConstructor(max_)));
  
  if (mpc_)
  {
    config.set("observation_min", extend(config["observation_min"].v(), VectorConstructor(0.)));
    config.set("observation_max", extend(config["observation_max"].v(), VectorConstructor(T_)));
  }
  
  config.set("reward_min", -sqrt(570));
  config.set("reward_max", 1.3);
}

void ReactorTrackingTask::start(int test, Vector *state)
{
  ReactorTask::start(test, state);

  test_ = test;

  if (!test_)
  {
    // Create timeline
    std::vector<double> t(setpoints_);
    for (size_t ii=0; ii != t.size(); ++ii)
      t[ii] = RandGen::getUniform(0, T_);
    std::sort(t.begin(), t.end());
    t[0] = 0.;
    
    timeline_ = Matrix(setpoints_, 2);
    for (size_t ii=0; ii != t.size(); ++ii)
    {
      timeline_(ii, 0) = t[ii];
      timeline_(ii, 1) = RandGen::getUniform(min_, max_);  
    }
  }
}

void ReactorTrackingTask::evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const
{
  if (state.size() != 5 || action.size() != 2 || next.size() != 5)
  {
    ERROR("Got dimensionality " << state.size() << ", " << action.size() << ", " << next.size() << ", expected 5, 2, 5");
    throw Exception("task/reactor/tracking requires dynamics/reactor");
  }

  // Maximize Fin while keeping Cb at setpoint  
  if (mpc_)
    *reward = -fin_weight_*pow(1-action[0]/700., 2) - (1-fin_weight_)*pow(setpoint(state[4])-state[1], 2) - 1e-10*pow(action[1], 2);
  else
    *reward = fin_weight_*(action[0]/700.) - (1-fin_weight_)*fabs(setpoint(state[4])-state[1]);
  
  // Normalize reward per timestep.
  *reward *= (next[4]-state[4]);
}

void ReactorTrackingTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  if (state.size() != 5)
  {
    ERROR("Got " << state.size() << " state dimensions, expected 5");
    throw Exception("task/reactor/tracking requires dynamics/reactor");
  }

  obs->v.resize(5 + mpc_);
  (*obs)[0] = state[0];
  (*obs)[1] = state[1];
  (*obs)[2] = state[2];
  (*obs)[3] = state[3];
  (*obs)[4] = setpoint(state[4]);
  if (mpc_)
    (*obs)[5] = state[4];

  obs->absorbing = false;
  
  if (state[4] > T_)
    *terminal = 1;
  else
    *terminal = 0;
}

bool ReactorTrackingTask::invert(const Observation &obs, Vector *state, double time) const
{
  state->resize(5);
  (*state)[0] = obs[0];
  (*state)[1] = obs[1];
  (*state)[2] = obs[2];
  (*state)[3] = obs[3];
  if (mpc_)
    (*state)[4] = obs[5];
  else    
    (*state)[4] = time;

  return true;
}

double ReactorTrackingTask::setpoint(double time) const
{
  if (test_)
  {
    Vector dummy;
    return profile_->read(VectorConstructor(time), &dummy);
  }
  else
  {
    size_t idx;
    for (idx=1; idx < timeline_.rows() && timeline_(idx, 0) < time; ++idx);
    return timeline_(idx-1, 1);
  }
}

Matrix ReactorTrackingTask::rewardHessian(const Vector &state, const Action &action) const
{
  Vector qr = -VectorConstructor(0, (1-fin_weight_), 0, 0, 0, 0, fin_weight_*pow(1./700, 2), 1e-10);
  return diagonal(qr);
}

// ReactorMaximizationTask

void ReactorMaximizationTask::request(ConfigurationRequest *config)
{
  ReactorTask::request(config);

  config->push_back(CRP("fin_weight", "Relative weight of Fin maximization", fin_weight_, CRP::Configuration, 0., DBL_MAX));
}

void ReactorMaximizationTask::configure(Configuration &config)
{
  ReactorTask::configure(config);

  fin_weight_ = config["fin_weight"];

  config.set("reward_min", 0);
  config.set("reward_max", 2.3);
}

void ReactorMaximizationTask::evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const
{
  if (state.size() != 5 || action.size() != 2 || next.size() != 5)
    throw Exception("task/reactor/balancing requires dynamics/reactor");

  // Maximize Cb and Fin at the same time
  *reward = (1-fin_weight_) * state[1] + fin_weight_*(action[0]/700.);
  
  // Normalize reward per timestep.
  *reward *= (next[4]-state[4]);
}
