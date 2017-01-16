/** \file cart_pole.h
 * \brief Cart-pole environment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-02
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
 
#ifndef GRL_CART_POLE_ENVIRONMENT_H_
#define GRL_CART_POLE_ENVIRONMENT_H_

#include <grl/environment.h>

namespace grl
{

/// Cart-Pole dynamics
/**
 * Based on A. Barto, R. Sutton, and C. Anderson,
 * “Neuronlike adaptive elements that can solve difficult learning control problems,”
 * IEEE T. Syst. Man Cy., vol. 13, no. 5, 1983.
 */
class CartPoleDynamics : public Dynamics
{
  public:
    TYPEINFO("dynamics/cart_pole", "Cart-pole dynamics from Barto et al.")

  public:
    double g_, mass_cart_, mass_pole_, total_mass_, length_,pole_mass_length_, force_mag_, tau_;
  
  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Dynamics
    virtual void eom(const Vector &state, const Vector &actuation, Vector *xd) const;
};

/// Cart-Pole swing-up task.
class CartPoleSwingupTask : public Task
{
  public:
    TYPEINFO("task/cart_pole/swingup", "Cart-pole swing-up task")
  
  public:
    double T_;
    int shaping_, randomization_, end_stop_penalty_;
    double gamma_;
  
  public:
    CartPoleSwingupTask() : T_(9.99), shaping_(0), randomization_(0), end_stop_penalty_(1), gamma_(1.0) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void start(int test, Vector *state) const;
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const;
    virtual bool invert(const Observation &obs, Vector *state) const;
    virtual Matrix rewardHessian(const Vector &state, const Action &action) const;
    
  protected:
    bool succeeded(const Vector &state) const;
    bool failed(const Vector &state) const;
    double potential(const Vector &state) const;
};

/// Cart-Pole balancing task.
class CartPoleBalancingTask : public Task
{
  public:
    TYPEINFO("task/cart_pole/balancing", "Cart-pole balancing task")
  
  public:
    double T_;
  
  public:
    CartPoleBalancingTask() : T_(9.99) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void start(int test, Vector *state) const;
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const;
    virtual bool invert(const Observation &obs, Vector *state) const;
    
  protected:
    bool failed(const Vector &state) const;
};

/// Cart-Pole balancing task with quadratic costs.
class CartPoleRegulatorTask : public RegulatorTask
{
  public:
    TYPEINFO("task/cart_pole/regulator", "Cart-pole regulator task")
  
  public:
    double T_;
  
  public:
    CartPoleRegulatorTask() : T_(9.99)
    {
      start_ = VectorConstructor(0, 0, 0, 0);
      goal_ = VectorConstructor(0, 0, 0, 0);
      stddev_ = VectorConstructor(0.1, 0.1, 0, 0);
      q_ = VectorConstructor(1, 1, 0, 0);
      r_ = VectorConstructor(0.01);
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual bool invert(const Observation &obs, Vector *state) const;
};

}

#endif /* GRL_CART_POLE_ENVIRONMENT_H_ */
