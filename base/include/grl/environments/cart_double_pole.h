/** \file cart_double_pole.h
 * \brief Cart-double-pole environment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-11-08
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Wouter Caarls
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
 
#ifndef GRL_CART_DOUBLE_POLE_ENVIRONMENT_H_
#define GRL_CART_DOUBLE_POLE_ENVIRONMENT_H_

#include <grl/environment.h>

namespace grl
{

class CartDoublePoleDynamics : public Dynamics
{
  public:
    TYPEINFO("dynamics/cart_double_pole", "Cart-double-pole dynamics from Zhong and Rock")

  public:
    double m_, m1_, m2_, l1_, l2_, b_, g_, L1_, L2_;
    double h1_, h2_, h3_, h4_, h5_, h6_, h7_, h8_;
  
  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Dynamics
    virtual CartDoublePoleDynamics *clone() const;
    virtual void eom(const Vector &state, const Vector &action, Vector *xd) const;
};

/// Cart-Pole swing-up task.
class CartDoublePoleSwingupTask : public Task
{
  public:
    TYPEINFO("task/cart_double_pole/swingup", "Cart-double-pole swing-up task")
  
  public:
    double T_;
  
  public:
    CartDoublePoleSwingupTask() : T_(9.99) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual CartDoublePoleSwingupTask *clone() const;
    virtual void start(int test, Vector *state) const;
    virtual void observe(const Vector &state, Vector *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const;
    virtual bool invert(const Vector &obs, Vector *state) const;
    virtual Matrix rewardHessian(const Vector &state, const Vector &action) const;
};

/// Cart-Pole balancing task.
class CartDoublePoleBalancingTask : public Task
{
  public:
    TYPEINFO("task/cart_double_pole/balancing", "Cart-double-pole balancing task")
  
  public:
    double T_;
  
  public:
    CartDoublePoleBalancingTask() : T_(9.99) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual CartDoublePoleBalancingTask *clone() const;
    virtual void start(int test, Vector *state) const;
    virtual void observe(const Vector &state, Vector *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const;
    virtual bool invert(const Vector &obs, Vector *state) const;
    
  protected:
    bool failed(const Vector &state) const;
};

/// Cart-Pole balancing task with quadratic costs.
class CartDoublePoleRegulatorTask : public RegulatorTask
{
  public:
    TYPEINFO("task/cart_double_pole/regulator", "Cart-double-pole regulator task")
  
  public:
    double T_;
  
  public:
    CartDoublePoleRegulatorTask() : T_(9.99)
    {
      start_ = VectorConstructor(0, 0, 0, 0, 0, 0);
      goal_ = VectorConstructor(0, 0, 0, 0, 0, 0);
      stddev_ = VectorConstructor(0.01, 0.01, 0.01, 0, 0, 0);
      q_ = VectorConstructor(1, 1, 1, 0, 0, 0);
      r_ = VectorConstructor(0.01);
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual CartDoublePoleRegulatorTask *clone() const;
    virtual void observe(const Vector &state, Vector *obs, int *terminal) const;
    virtual bool invert(const Vector &obs, Vector *state) const;
};

}

#endif /* GRL_CART_DOUBLE_POLE_ENVIRONMENT_H_ */
