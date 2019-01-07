/** \file breakout.h
 * \brief Breakout environment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2019-01-07
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
 
#ifndef GRL_BREAKOUT_ENVIRONMENT_H_
#define GRL_BREAKOUT_ENVIRONMENT_H_

#include <Box2D/Box2D.h>
#include <grl/environment.h>
#include <grl/visualization.h>

namespace grl
{

/// Breakout sandbox
class BreakoutSandbox : public Sandbox
{
  public:
    TYPEINFO("sandbox_model/breakout", "Breakout sandbox")

  public:
    b2World *world_;
    b2Body *environment_, *cart_, *paddle_, *ball_;
    b2PrismaticJoint *piston_;
    double tau_;
    size_t steps_;
    
    double time_;

  public:
    BreakoutSandbox() : world_(NULL), environment_(NULL), cart_(NULL), paddle_(NULL), ball_(NULL), piston_(NULL), tau_(0.05), steps_(5) ,time_(0) { }
    ~BreakoutSandbox()
    {
      safe_delete(&world_);
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Sandbox
    virtual void start(const Vector &hint, Vector *state);
    virtual double step(const Vector &actuation, Vector *next);
};

/// Breakout targeting task.
class BreakoutTargetingTask : public Task
{
  public:
    TYPEINFO("task/breakout/targeting", "Target a single point")
    
  public:
    double timeout_, randomization_;

  public:
    BreakoutTargetingTask() : timeout_(10), randomization_(0) { }
  
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

/// Breakout visualization.
class BreakoutVisualization : public Visualization
{
  public:
    TYPEINFO("visualization/breakout", "Breakout visualization")

  protected:
    VectorSignal *state_;

  public:
    BreakoutVisualization() : state_(NULL) { }

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

#endif /* GRL_BREAKOUT_ENVIRONMENT_H_ */
