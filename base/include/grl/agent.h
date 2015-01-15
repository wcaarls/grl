/*
 * agent.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef AGENT_H_
#define AGENT_H_

#include <grl/configurable.h>

namespace grl
{

/// Interacts with an Environment.
class Agent : public Configurable
{
  public:
    virtual ~Agent() { }
    virtual Agent *clone() const = 0;
    virtual void start(const Vector &obs, Vector *action) = 0;
    virtual void step(const Vector &obs, double reward, Vector *action) = 0;
    virtual void end(double reward) = 0;
};

}

#endif /* AGENT_H_ */
