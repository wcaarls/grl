/*
 * experiment.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef EXPERIMENT_H_
#define EXPERIMENT_H_

#include <grl/configurable.h>
#include <grl/agent.h>
#include <grl/environment.h>

namespace grl
{

/// Runs an experiment.
class Experiment : public Configurable
{
  public:
    virtual ~Experiment() { }
    virtual Experiment *clone() const = 0;
    virtual void run() const = 0;
};

/// Standard Agent-Environment interaction experiment.
class OnlineLearningExperiment : public Experiment
{
  public:
    TYPEINFO("experiment/online_learning")

  protected:
    Agent *agent_;
    Environment *environment_;
    
    size_t runs_, trials_, steps_;
    double rate_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Experiment
    virtual OnlineLearningExperiment *clone() const;
    virtual void run() const;  
};

}

#endif /* EXPERIMENT_H_ */
