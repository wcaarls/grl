/** \file tlm.h
 * \brief Two-link manipulator environment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-12
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
 
#ifndef GRL_TLM_ENVIRONMENT_H_
#define GRL_TLM_ENVIRONMENT_H_

#include <grl/signal.h>
#include <grl/environment.h>
#include <grl/visualization.h>

namespace grl
{

/// TwoLinkManipulator dynamics.
class TwoLinkManipulatorDynamics : public Dynamics
{
  public:
    TYPEINFO("dynamics/tlm", "Two-link manipulator dynamics")
    
    enum StateDimIndex {siAngle1, siAngle2, siAngleRate1, siAngleRate2, siTime};
    
  public:
    double p1_, p2_, p3_;
    double damping1_, damping2_;

  public:
    TwoLinkManipulatorDynamics() : damping1_(0.08), damping2_(0.02)
    {
      double l = 0.4;
      double m1 = 1.25, m2 = 0.8, I1 = 0.066, I2 = 0.043, c1 = 0.2, c2 = 0.2;
      
      p1_ = m1*c1*c1 + m2*l*l + I1;
      p2_ = m2*c2*c2 + I2;
      p3_ = m2*l*c2;
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Dynamics
    virtual void eom(const Vector &state, const Vector &actuation, Vector *xd) const;
};

/// TwoLinkManipulator swing-up task.
class TwoLinkManipulatorBalancingTask : public Task
{
  public:
    TYPEINFO("task/tlm/balancing", "Two-link manipulator balancing task")
  
  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void start(int test, Vector *state) const;
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const;
    virtual bool invert(const Observation &obs, Vector *state) const;
};

/// Two-link manipulator visualization.
class TwoLinkManipulatorVisualization : public Visualization
{
  public:
    TYPEINFO("visualization/tlm", "Two-link manipulator visualization")

  protected:
    VectorSignal *state_;
  
  public:
    TwoLinkManipulatorVisualization() : state_(NULL) { }
    
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Visualization
    virtual void draw();
    virtual void idle();
    virtual void reshape(int width, int height);
};

}

#endif /* GRL_TLM_ENVIRONMENT_H_ */
