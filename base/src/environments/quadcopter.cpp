/** \file quadcopter.cpp
 * \brief Quadcopter dynamics source file
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2020-04-14
 *
 * \copyright \verbatim
 * Copyright(c) 2017 Abhijit Majumdar
 * Copyright(c) 2020 Wouter Caarls
 *
 * Based on
 * https://github.com/abhijitmajumdar/Quadcopter_simulator/blob/master/quadcopter.py
 * which carries the following license:
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files(the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions :
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <grl/environments/quadcopter.h>

using namespace grl;
using namespace Eigen;

REGISTER_CONFIGURABLE(QuadcopterDynamics)
REGISTER_CONFIGURABLE(QuadcopterRegulatorTask)

void QuadcopterDynamics::request(ConfigurationRequest *config)
{
  config->push_back(CRP("g", "Gravitational constant", g_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("b", "Drag coefficient", b_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("L", "Propeller distance from center", L_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("r", "Central body radius", r_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("M", "Central body weight", weight_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("d", "Propeller diameter", prop_dia_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("p", "Propeller pitch", prop_pitch_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("m", "Motor weight", prop_weight_, CRP::Configuration, 0., DBL_MAX));

  config->push_back(CRP("limits", "vector.limits/quadcopter", "Position and velocity limits (0=unlimited)", limits_, CRP::Configuration));
}

void QuadcopterDynamics::configure(Configuration &config)
{
  g_ = config["g"];
  b_ = config["b"];
  L_ = config["L"];
  r_ = config["r"];
  weight_ = config["M"];
  prop_dia_ = config["d"];
  prop_pitch_ = config["p"];
  prop_weight_ = config["m"];

  limits_ = config["limits"].v();
  if (limits_.size() != 2)
    throw bad_param("dynamics/quadcopter:limits");
  
  for (size_t ii=0; ii != 4; ++ii)
    propellers_.push_back(Propeller(prop_dia_, prop_pitch_));

  // From Quadrotor Dynamics and Control by Randal Beard
  double ixx = ((2 * weight_ * r_ * r_) / 5) + (2 * prop_weight_ * L_ * L_);
  double iyy = ixx;
  double izz = ((2 * weight_ * r_ * r_) / 5) + (4 * prop_weight_ * L_ * L_);

  I_ << ixx, 0, 0, 0, iyy, 0, 0, 0, izz;
  invI_ = I_.inverse();
}

void QuadcopterDynamics::reconfigure(const Configuration &config)
{
}

void QuadcopterDynamics::eom(const Vector &state, const Vector &actuation, Vector *xd) const
{
  // State space representation : [x y z x_dot y_dot z_dot theta phi gamma theta_dot phi_dot gamma_dot]
  // From Quadcopter Dynamics, Simulation, and Control by Andrew Gibiansky
  if (state.size() != 13 || actuation.size() != 4)
    throw Exception("dynamics/quadcopter requires a task/quadcopter subclass");

  double t[4];
  for (size_t ii = 0; ii != 4; ++ii)
    t[ii] = propellers_[ii].getThrust(actuation[ii]);
    
  xd->resize(13);

  // The velocities(t + 1 x_dots equal the t x_dots)
  (*xd)[0] = state[3];
  (*xd)[1] = state[4];
  (*xd)[2] = state[5];

  // The acceleration
  Vector3d x_dotdot = Vector3d(0, 0, -weight_*g_) +
                      rotationMatrix(state[6], state[7], state[8]) * Vector3d(0, 0, t[0]+t[1]+t[2]+t[3]) / weight_;
  (*xd)[3] = x_dotdot[0];
  (*xd)[4] = x_dotdot[1];
  (*xd)[5] = x_dotdot[2];

  // The angular rates(t + 1 theta_dots equal the t theta_dots)
  (*xd)[6] = state[9];
  (*xd)[7] = state[10];
  (*xd)[8] = state[11];

  // The angular accelerations (small angle approximation for omega)
  Vector3d omega = Vector3d(state[9], state[10], state[11]);
  Vector3d tau = Vector3d(L_ * (t[0] - t[2]), L_ * (t[1] - t[3]), b_ * (t[0] - t[1] + t[2] - t[3]));
  Vector3d omega_dot = invI_ * (tau - omega.cross(I_*omega));

  (*xd)[9] = omega_dot[0];
  (*xd)[10] = omega_dot[1];
  (*xd)[11] = omega_dot[2];

  // Time
  (*xd)[12] = 1;
  
  // Limit position
  if (limits_[0])
    for (size_t ii=0; ii != 3; ++ii)
    {
      if (state[ii] > limits_[0])
      {
        if ((*xd)[0+ii] > 0) (*xd)[0+ii] = 0;
        if ((*xd)[3+ii] > 0) (*xd)[3+ii] = 0;
      }
      if (state[ii] < -limits_[0])
      {
        if ((*xd)[0+ii] < 0) (*xd)[0+ii] = 0;
        if ((*xd)[3+ii] < 0) (*xd)[3+ii] = 0;
      }
    }

  // Limit velocities
  if (limits_[1])
    for (size_t ii=0; ii != 3; ++ii)
    {
      if ((*xd)[ii] > limits_[1])
      {
        (*xd)[ii] = limits_[1];
        if ((*xd)[3+ii] > 0) (*xd)[3+ii] = 0;
      }
      if ((*xd)[ii] < -limits_[1])
      {
        (*xd)[ii] = -limits_[1];
        if ((*xd)[3+ii] < 0) (*xd)[3+ii] = 0;
      }
    }
}

Matrix3d QuadcopterDynamics::rotationMatrix(double theta, double phi, double gamma) const
{
  double ct = cos(theta);
  double cp = cos(phi);
  double cg = cos(gamma);
  double st = sin(theta);
  double sp = sin(phi);
  double sg = sin(gamma);
  Matrix3d R_x, R_y, R_z;

  // ZYX (321) form  
  R_x << 1, 0, 0, 0, ct, -st, 0, st, ct;
  R_y << cp, 0, sp, 0, 1, 0, -sp, 0, cp;
  R_z << cg, -sg, 0, sg, cg, 0, 0, 0, 1;
  
  return R_z * (R_y * R_x);
}

// Regulator

void QuadcopterRegulatorTask::request(ConfigurationRequest *config)
{
  RegulatorTask::request(config);
  
  config->push_back(CRP("action_range", "Range of allowed actions (rpm)", action_range_, CRP::Configuration));
  config->push_back(CRP("limits", "vector.limts/quadcopter", "Position and velocity limits (0=unlimited)", limits_, CRP::Configuration));
  config->push_back(CRP("wrap", "Wrap positions around limits", wrap_, CRP::Configuration, 0, 1));
  config->push_back(CRP("penalty", "Terminal penalty for crossing position limit", penalty_, CRP::Configuration, 0., DBL_MAX));
}

void QuadcopterRegulatorTask::configure(Configuration &config)
{
  RegulatorTask::configure(config);
  
  action_range_ = config["action_range"].v();
  if (action_range_.size() == 1)
    action_range_ = VectorConstructor(0, action_range_[0]);
  if (action_range_.size() != 2)
    throw bad_param("task/quadcopter/regulator:action_range");
    
  limits_ = config["limits"].v();
  if (limits_.size() != 2)
    throw bad_param("dynamics/quadcopter:limits");
    
  wrap_ = config["wrap"];
  penalty_ = config["penalty"];
  
  if (wrap_ && !limits_[0])
  {
    ERROR("Cannot wrap without valid position limit");
    throw bad_param("dynamics/quadcopter:{limits,wrap}");
  }
  
  if (q_.size() != 12)
    throw bad_param("task/quadcopter/regulator:q");
  if (r_.size() != 4)
    throw bad_param("task/quadcopter/regulator:r");
    
  double p = limits_[0]?limits_[0]:1, v = limits_[1]?limits_[1]:10;

  config.set("observation_min", VectorConstructor(-p, -p, -p, -M_PI, -M_PI, -M_PI, -v, -v, -v, -10*M_PI, -10*M_PI, -10*M_PI));
  config.set("observation_max", VectorConstructor( p,  p,  p,  M_PI,  M_PI,  M_PI,  v,  v,  v,  10*M_PI,  10*M_PI,  10*M_PI));
  config.set("action_min", VectorConstructor(action_range_[0], action_range_[0], action_range_[0], action_range_[0]));
  config.set("action_max", VectorConstructor(action_range_[1], action_range_[1], action_range_[1], action_range_[1]));
}

void QuadcopterRegulatorTask::reconfigure(const Configuration &config)
{
  RegulatorTask::reconfigure(config);
}

void QuadcopterRegulatorTask::evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const
{
  // Bound errors
  Vector _state = wrap(state), _next = wrap(next);
  
  RegulatorTask::evaluate(_state, action, _next, reward);
  
  if (failed(next))
    *reward -= penalty_;
}

void QuadcopterRegulatorTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  RegulatorTask::observe(state, obs, terminal);

  if (state.size() != 13)
    throw Exception("task/quadcopter/regulator requires dynamics/quadcopter");

  Vector _state = wrap(state);
    
  obs->v.resize(12);
  for (size_t ii=0; ii < 12; ++ii)
    (*obs)[ii] = _state[ii];
  
  if (failed(state))
  {
    obs->absorbing = true;
    *terminal = 2;
  }
  else
    obs->absorbing = false;
}

bool QuadcopterRegulatorTask::invert(const Observation &obs, Vector *state, double time) const
{
  *state = extend(obs, VectorConstructor(time));
  
  return true;
}

Vector QuadcopterRegulatorTask::wrap(const Vector &state) const
{
  Vector _state = state;

  for (size_t ii=0; ii != 3; ++ii)
  {    
    // Simulate 3D torus
    if (wrap_)
    {
      double p = fmod(state[ii]+limits_[0], 2*limits_[0]);
      if (p < 0) p += 2*limits_[0];
      p -= limits_[0];
      _state[ii] = p;
    }

    // Bound angles between -pi, pi
    double a = fmod(state[3+ii]+M_PI, 2*M_PI);
    if (a < 0) a += 2*M_PI;
    a -= M_PI;
    _state[3+ii] = a;
  }  
  
  return _state;
}

bool QuadcopterRegulatorTask::failed(const Vector &state) const
{
  if (penalty_ && (fabs(state[0]) > limits_[0] || 
                   fabs(state[1]) > limits_[0] ||
                   fabs(state[2]) > limits_[0]))
    return true;
  else
    return false;
}
