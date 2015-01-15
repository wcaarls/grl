/*
 * black_box_agent.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef BLACK_BOX_AGENT_H_
#define BLACK_BOX_AGENT_H_

#include <grl/agent.h>
#include <grl/policy.h>

namespace grl
{

/// Black-box learning agent.
class BlackBoxAgent : public Agent
{
  public:
    TYPEINFO("agent/black_box")

  protected:
    Policy *policy_;
    size_t ii;
};

}

#endif /* BLACK_BOX_AGENT_H_ */
