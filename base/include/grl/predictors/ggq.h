/*
 * ggq.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef PREDICTOR_GGQ_H_
#define PREDICTOR_GGQ_H_

#include <grl/configurable.h>
#include <grl/predictor.h>
#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/trace.h>
#include <grl/policy.h>
#include <grl/policies/q.h>
#include <grl/sampler.h>

namespace grl
{

/// Off-policy value function predictor that converges with function approximation
class GGQPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/ggq")

  protected:
    double alpha_, eta_, gamma_, lambda_;
    Projector *projector_;
    Representation *theta_, *w_;
    Policy *policy_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Predictor
    virtual GGQPredictor *clone() const;
    virtual void update(const Transition &transition);
    virtual void finalize();
};

}

#endif /* PREDICTOR_GGQ_H_ */
