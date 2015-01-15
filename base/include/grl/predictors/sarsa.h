/*
 * predictor.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef PREDICTOR_SARSA_H_
#define PREDICTOR_SARSA_H_

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

/// Value function predictor using the taken action.
class SARSAPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/sarsa")

  protected:
    double alpha_, gamma_, lambda_;
    Projector *projector_;
    Representation *representation_;
    Trace *trace_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(const Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Predictor
    virtual SARSAPredictor *clone() const;
    virtual void update(const Transition &transition);
    virtual void finalize();
};

/// Value function predictor using the distribution of a QPolicy.
class ExpectedSARSAPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/expected_sarsa")

  protected:
    double alpha_, gamma_, lambda_;
    Projector *projector_;
    Representation *representation_;
    QPolicy *policy_;
    Sampler *sampler_;
    Trace *trace_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(const Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Predictor
    virtual ExpectedSARSAPredictor *clone() const;
    virtual void update(const Transition &transition);
    virtual void finalize();
};

}

#endif /* PREDICTOR_SARSA_H_ */
