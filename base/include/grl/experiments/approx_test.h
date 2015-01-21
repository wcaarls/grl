/*
 * online_learning.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef GRL_APPROX_TEST_EXPERIMENT_H_
#define GRL_APPROX_TEST_EXPERIMENT_H_

#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/experiment.h>

namespace grl
{

/// Standard Agent-Environment interaction experiment.
class ApproxTestExperiment : public Experiment
{
  public:
    TYPEINFO("experiment/approx_test")

  protected:
    Projector *projector_;
    Representation *representation_;
    Mapping *mapping_;
    
    Vector min_, max_;
    size_t outputs_, train_samples_, test_samples_;
    std::string file_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Experiment
    virtual ApproxTestExperiment *clone() const;
    virtual void run() const;  
};

}

#endif /* GRL_APPROX_TEST_EXPERIMENT_H_ */
