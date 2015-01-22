/*
 * dyna_agent.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef DYNA_AGENT_H_
#define DYNA_AGENT_H_

#include <grl/agent.h>
#include <grl/policy.h>
#include <grl/predictor.h>
#include <grl/projector.h>
#include <grl/representation.h>

namespace grl
{

/// Dyna learning agent.
class DynaAgent : public Agent
{
  public:
    TYPEINFO("agent/dyna")

  protected:
    Policy *policy_;
    Predictor *predictor_;
    Agent *model_agent_;
    Projector *model_projector_;
    Representation *model_representation_;
    
    Vector prev_obs_, prev_action_, wrapping_;
    size_t planning_steps_;
    
  public:
    DynaAgent() : policy_(NULL), predictor_(NULL), model_agent_(NULL), model_projector_(NULL), model_representation_(NULL), planning_steps_(1) { }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual DynaAgent *clone() const;
    virtual void start(const Vector &obs, Vector *action);
    virtual void step(const Vector &obs, double reward, Vector *action);
    virtual void end(double reward);
    
  protected:
    void learnModel(const Transition &t);
    void stepModel(const Vector &obs, const Vector &action, Vector *next, double *reward, int *terminal);
    void runModel();
  
};

}

#endif /* DYNA_AGENT_H_ */
