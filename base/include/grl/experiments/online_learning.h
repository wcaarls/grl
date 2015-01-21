/*
 * online_learning.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef GRL_ONLINE_LEARNING_EXPERIMENT_H_
#define GRL_ONLINE_LEARNING_EXPERIMENT_H_

#include <grl/agent.h>
#include <grl/environment.h>
#include <grl/experiment.h>

namespace grl
{

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

#endif /* GRL_ONLINE_LEARNING_EXPERIMENT_H_ */
