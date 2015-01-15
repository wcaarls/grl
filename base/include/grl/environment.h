/*
 * environment.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include <grl/configurable.h>
#include <grl/policy.h>

namespace grl
{

/// Real or simulated world.
class Environment : public Configurable
{
  public:
    virtual ~Environment() { }
    virtual Environment *clone() const = 0;

    virtual void start(Vector *obs) = 0;
    virtual void step(const Vector &action, Vector *obs, double *reward, int *terminal) = 0;
};

/// Random-access transition model (works on states instead of observations).
class Model : public Configurable
{
  public:
    virtual ~Model() { }
    virtual Model *clone() const = 0;

    virtual void step(const Vector &state, const Vector &action, Vector *next) const = 0;
};

class Task : public Configurable
{
  public:
    virtual ~Task() { }
    virtual Task *clone() const = 0;

  public:
    virtual void start(Vector *state) const = 0;
    virtual void observe(const Vector &state, Vector *obs, int *terminal) const = 0;
    virtual bool evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const = 0;
};

/// Environment that uses a transition model internally.
class ModeledEnvironment : public Environment
{
  public:
    TYPEINFO("environment/modeled")

  public:
    Model *model_;
    Task *task_;
    Vector state_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Environment
    virtual ModeledEnvironment *clone() const;
    virtual void start(Vector *obs);
    virtual void step(const Vector &action, Vector *obs, double *reward, int *terminal);
};

/// Equations of motion.
class Dynamics : public Configurable
{
  public:
    virtual ~Dynamics() { }
    virtual Dynamics *clone() const = 0;

    virtual void eom(const Vector &state, const Vector &action, Vector *xdd) const = 0;
};

class DynamicalModel : public Model
{
  public:
    TYPEINFO("model/dynamical")
    
  public:
    Dynamics *dynamics_;
    double tau_;
    size_t steps_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Model
    virtual DynamicalModel *clone() const;
    virtual void step(const Vector &state, const Vector &action, Vector *next) const;
};

}

#endif /* ENVIRONMENT_H_ */
