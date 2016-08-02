/** \file environment.h
 * \brief Generic and basic environment definitions.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
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

#ifndef GRL_ENVIRONMENT_H_
#define GRL_ENVIRONMENT_H_

#include <grl/configurable.h>
#include <grl/state.h>
#include <grl/policy.h>
#include <grl/exporter.h>
#include <grl/mapping.h>

namespace grl
{

/// Real or simulated world.
class Environment : public Configurable
{
  public:
    virtual ~Environment() { }
    virtual Environment *clone() const = 0;

    /// Start the environment, returning the first observation.
    virtual void start(int test, Vector *obs) = 0;
    
    /// Take a step, returning the next observation, reward, and whether the episode terminated.
    virtual double step(const Vector &action, Vector *obs, double *reward, int *terminal) = 0;

    /// Progress report.
    virtual void report(std::ostream &os) { }
};

/// Random-access transition model (works on states instead of observations).
class Model : public Configurable
{
  public:
    virtual ~Model() { }
    virtual Model *clone() const = 0;
    virtual double step(const Vector &state, const Vector &action, Vector *next) const = 0;
};

class Task : public Configurable
{
  public:
    virtual ~Task() { }
    virtual Task *clone() const = 0;

  public:
    virtual void request(ConfigurationRequest *config)
    {
      config->push_back(CRP("observation_dims", "int.observation_dims", "Number of observation dimensions", CRP::Provided));
      config->push_back(CRP("observation_min", "vector.observation_min", "Lower limit on observations", CRP::Provided));
      config->push_back(CRP("observation_max", "vector.observation_max", "Upper limit on observations", CRP::Provided));
      config->push_back(CRP("action_dims", "int.action_dims", "Number of action dimensions", CRP::Provided));
      config->push_back(CRP("action_min", "vector.action_min", "Lower limit on actions", CRP::Provided));
      config->push_back(CRP("action_max", "vector.action_max", "Upper limit on actions", CRP::Provided));
      config->push_back(CRP("reward_min", "double.reward_min", "Lower limit on immediate reward", CRP::Provided));
      config->push_back(CRP("reward_max", "double.reward_max", "Upper limit on immediate reward", CRP::Provided));
    }    
  
    /// Start the task, returning the initial state.
    virtual void start(int test, Vector *state) const = 0;
    
    /// Observe a state, returning the observation and whether the episode ended.
    virtual void observe(const Vector &state, Vector *obs, int *terminal) const = 0;
    
    /// Evaluate a state transition, returning the reward.
    virtual void evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const = 0;
    
    /**
     * \brief Invert the state->observation projection.
     *
     * Returns false if not implemented by a specific task.
     */
    virtual bool invert(const Vector &obs, Vector *state) const { return false; }
    
    /// Returns the Hessian of the reward around the given state and action.
    virtual Matrix rewardHessian(const Vector &state, const Vector &action) const
    {
      return Matrix();
    }
};

/// Task that regulates to a goal state with quadratic cost.
class RegulatorTask : public Task
{
  protected:
    Vector start_, goal_, stddev_, q_, r_;

  public:
    
    void request(ConfigurationRequest *config)
    {
      Task::request(config);
      
      config->push_back(CRP("start", "Starting state", start_));
      config->push_back(CRP("goal", "Goal state", goal_));
      config->push_back(CRP("stddev", "Starting state standard deviation", stddev_));
      config->push_back(CRP("q", "Q (state cost) matrix diagonal", q_));
      config->push_back(CRP("r", "R (action cost) matrix diagonal", r_));
    }

    void configure(Configuration &config)
    {
      start_ = config["start"].v();
      goal_ = config["goal"].v();
      stddev_ = config["stddev"].v();
      q_ = config["q"].v();
      r_ = config["r"].v();

      if (!start_.size())
        throw bad_param("task/regulator:start");
      
      if (start_.size() != goal_.size())
        throw bad_param("task/regulator:{start,goal}");

      if (!stddev_.size())
        stddev_ = ConstantVector(start_.size(), 0.);

      if (start_.size() != stddev_.size())
        throw bad_param("task/regulator:stddev");
        
      if (start_.size() != q_.size())
        throw bad_param("task/regulator:q");
        
      if (!r_.size())
        throw bad_param("task/regulator:r");
      
      config.set("observation_dims", q_.size());
      config.set("action_dims", r_.size());
      config.set("reward_min", -1000);
      config.set("reward_max", 0);
    }

    void start(int test, Vector *state) const
    {
      *state = ConstantVector(start_.size()+1, 0.);

      for (size_t ii=0; ii < stddev_.size(); ++ii)
        (*state)[ii] = RandGen::getNormal(start_[ii], test?0.:stddev_[ii]);
    }

    void evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
    {
      if (state.size() != q_.size()+1 || action.size() != r_.size() || next.size() != q_.size()+1)
        throw Exception("Unexpected state or action size");

      *reward = 0.;
      
      for (size_t ii=0; ii < q_.size(); ++ii)
        *reward -= 0.5*q_[ii]*pow(state[ii]-goal_[ii], 2);
      for (size_t ii=0; ii < r_.size(); ++ii)
        *reward -= 0.5*r_[ii]*pow(action[ii], 2);
    }

    Matrix rewardHessian(const Vector &state, const Vector &action) const
    {
      return diagonal(-extend(q_, r_));
    }
};

/// Environment that uses a transition model internally.
class ModeledEnvironment : public Environment
{
  public:
    TYPEINFO("environment/modeled", "Environment that uses a state transition model internally")

  public:
    Model *model_;
    Task *task_;
    Vector state_, obs_;
    State *state_obj_;
    Exporter *exporter_;
    
    int test_;
    double time_test_, time_learn_;

  public:
    ModeledEnvironment() : model_(NULL), task_(NULL), state_obj_(NULL), exporter_(NULL), test_(false), time_test_(0.), time_learn_(0.) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Environment
    virtual ModeledEnvironment *clone() const;
    virtual void start(int test, Vector *obs);
    virtual double step(const Vector &action, Vector *obs, double *reward, int *terminal);
};

/// Equations of motion.
class Dynamics : public Configurable
{
  public:
    virtual ~Dynamics() { }
    virtual Dynamics *clone() const = 0;

    /// Compute equations of motion, returning accelerations.
    virtual void eom(const Vector &state, const Vector &action, Vector *xdd) const = 0;
    virtual void finalize(Vector &next) const {}
};

class DynamicalModel : public Model
{
  public:
    TYPEINFO("model/dynamical", "State transition model that integrates equations of motion")
    
  public:
    Dynamics *dynamics_;
    double tau_;
    size_t steps_;

  public:
    DynamicalModel() : dynamics_(NULL), tau_(0.05), steps_(5) { }
        
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Model
    virtual DynamicalModel *clone() const;
    virtual double step(const Vector &state, const Vector &action, Vector *next) const;
    virtual void finalize(Vector &next) const;
};

/// Environment modifier that adds sensor and actuator noise.
class NoiseEnvironment : public Environment
{
  public:
    TYPEINFO("environment/pre/noise", "Injects noise into an environment")

  public:
    Environment *environment_;
    
    Vector sensor_noise_, actuator_noise_;

  public:
    NoiseEnvironment() : environment_(NULL)
    {
      sensor_noise_ = VectorConstructor(0.);
      actuator_noise_ = VectorConstructor(0.);
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Environment
    virtual NoiseEnvironment *clone() const;
    virtual void start(int test, Vector *obs);
    virtual double step(const Vector &action, Vector *obs, double *reward, int *terminal);
};

/// Environment modifier that adds reward shaping.
class ShapingEnvironment : public Environment
{
  public:
    TYPEINFO("environment/pre/shaping", "Adds reward shaping to an environment")

  public:
    Environment *environment_;
    Mapping *shaping_function_;
    double gamma_;
    
    Vector prev_obs_;
    double total_reward_;

  public:
    ShapingEnvironment() : environment_(NULL), shaping_function_(NULL), gamma_(1.), total_reward_(0.)
    {
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Environment
    virtual ShapingEnvironment *clone() const;
    virtual void start(int test, Vector *obs);
    virtual double step(const Vector &action, Vector *obs, double *reward, int *terminal);
    virtual void report(std::ostream &os);
};

/// Sequential-access transition model.
class Sandbox : public Configurable
{
  public:
    virtual ~Sandbox() { }
    virtual Sandbox *clone() const = 0;

    virtual void start(const Vector &hint, Vector *state) = 0;
    virtual double step(const Vector &action, Vector *next) = 0;
  protected:
    Vector state_;
};

/// Sequential-access transition environment.
class SandboxEnvironment : public Environment
{
  public:
    TYPEINFO("environment/sandbox", "Non-Markov environment")

  public:
    Sandbox *sandbox_;
    Task *task_;
    Vector state_, obs_;
    State *state_obj_;
    Exporter *exporter_;

    int test_;
    double time_test_, time_learn_;

  public:
    SandboxEnvironment() : sandbox_(NULL), task_(NULL), state_obj_(NULL), exporter_(NULL), test_(false), time_test_(0.), time_learn_(0.) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Environment
    virtual SandboxEnvironment *clone() const;
    virtual void start(int test, Vector *obs);
    virtual double step(const Vector &action, Vector *obs, double *reward, int *terminal);
};

class SandboxDynamicalModel : public Sandbox
{
  public:
    TYPEINFO("sandbox_model/dynamical", "State transition model that integrates equations of motion and augments state vector with additional elements")

  public:
    SandboxDynamicalModel() : dof_count_(0) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    
    // From Model
    virtual SandboxDynamicalModel *clone() const;
    virtual void start(const Vector &hint, Vector *state);
    virtual double step(const Vector &action, Vector *next);
    
  private:
    DynamicalModel dm_;
    int dof_count_;
};


}

#endif /* GRL_ENVIRONMENT_H_ */
