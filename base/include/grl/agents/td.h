/*
 * td_agent.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef TD_AGENT_H_
#define TD_AGENT_H_

#include <grl/agent.h>
#include <grl/policy.h>
#include <grl/predictor.h>

namespace grl
{

/// Temporal difference learning agent.
class TDAgent : public Agent
{
  public:
    TYPEINFO("agent/td")

  protected:
    Policy *policy_;
    Predictor *predictor_;
    
    Vector prev_obs_, prev_action_;
    
  public:
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(const Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual TDAgent *clone() const;
    virtual void start(const Vector &obs, Vector *action);
    virtual void step(const Vector &obs, double reward, Vector *action);
    virtual void end(double reward);
};

}

#endif /* TD_AGENT_H_ */
