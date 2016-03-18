/** \file compass_walker.h
 * \brief Compass walker environment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-03-14
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

#ifndef GRL_COMPASS_WALKER_ENVIRONMENT_H_
#define GRL_COMPASS_WALKER_ENVIRONMENT_H_

#include <grl/environment.h>
#include <grl/environments/compass_walker/SWModel.h>

namespace grl
{

class CompassWalker
{
  public:
    enum stateIndex       { siStanceLegAngle, siHipAngle, siStanceLegAngleRate, siHipAngleRate,
                            siStanceLegChanged, siStanceFootX, siLastHipX, siHipVelocity, siTime,
                            siTimeout, ssStateSize};

    enum observationIndex { oiStanceLegAngle, oiHipAngle, oiStanceLegAngleRate, oiHipAngleRate,
                            oiStanceLegChanged, oiHipVelocity, osMaxObservationSize};
};

// Compass (simplest) walker model.
class CompassWalkerModel : public Model
{
  public:
    TYPEINFO("model/compass_walker", "Simplest walker model from Garcia et al.")
    
  protected:
    double tau_;
    double slope_angle_;
    size_t steps_;
    CSWModel model_;
    std::string integrator_out_;

  public:
    CompassWalkerModel() : tau_(0.2), steps_(20), slope_angle_(0.004) { }
    ~CompassWalkerModel() {}
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Model
    virtual CompassWalkerModel *clone() const;
    virtual double step(const Vector &state, const Vector &action, Vector *next) const;
};

// Compass (simplest) walker non-markov model.
class CompassWalkerSandbox : public Sandbox
{
  public:
    TYPEINFO("sandbox_model/compass_walker", "Simplest walker model from Garcia et al. with a sequential evaluation")

  protected:
    double tau_;
    double time_;
    int test_;
    double slope_angle_;
    size_t steps_;
    CSWModel model_;
    std::deque<double> hip_instant_velocity_;
    Vector state_;
    Exporter *exporter_;
    int use_avg_velocity_;

  public:
    CompassWalkerSandbox() : tau_(0.2), steps_(20), slope_angle_(0.004),
      test_(0), time_(0.0), use_avg_velocity_(1) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Model
    virtual CompassWalkerSandbox *clone() const;
    virtual void start(const Vector &hint, Vector *state);
    virtual double step(const Vector &action, Vector *next);
};

// Walk forward task for compass walker.
class CompassWalkerWalkTask : public Task
{
  public:
    TYPEINFO("task/compass_walker/walk", "Compass walker walking task")
  
  protected:
    double T_;
    bool verbose_;
    double initial_state_variation_;
    double slope_angle_;
    double neg_reward_;
    int observation_dims_;
    Vector observe_;          // Indicator vector with 1s for states, which are observed by an agent

  public:
    CompassWalkerWalkTask() : T_(100), initial_state_variation_(0.2), slope_angle_(0.004),
      neg_reward_(-100.0), verbose_(false), observe_(VectorConstructor(1, 1, 1, 1, 1, 0)){ }
    
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Task
    virtual CompassWalkerWalkTask *clone() const;
    virtual void start(int test, Vector *state) const;
    virtual void observe(const Vector &state, Vector *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const;
    virtual bool invert(const Vector &obs, Vector *state) const;
};

// Walk forward with a reference velocity task for compass walker.
class CompassWalkerVrefTask : public CompassWalkerWalkTask
{
  public:
    TYPEINFO("task/compass_walker/vref", "Compass walker tracking velocity task")

  protected:
    double vref_;

  public:
    CompassWalkerVrefTask() : vref_(0.12) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);

    // From Task
    virtual void start(int test, Vector *state) const;
    virtual void evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const;
};

// Walk forward with a reference velocity task for compass walker in limit cycle and control minimization.
class CompassWalkerVrefuTask : public CompassWalkerVrefTask
{
  public:
    TYPEINFO("task/compass_walker/vrefu", "Compass walker tracking velocity task with controls minimization")

  public:
    CompassWalkerVrefuTask() { }

    // From Task
    virtual void evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const;
};

}

#endif /* GRL_COMPASS_WALKER_ENVIRONMENT_H_ */
