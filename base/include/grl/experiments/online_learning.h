/** \file online_learning.h
 * \brief Online learning experiment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
 *
 * \copyright \verbatim
 * Copyright (c) 2015, Wouter Caarls
 * All rights reserved.
 *
 * This file is part of GRL, the Generic Reinforcement Learning library.
 *
 * GRL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * \endverbatim
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
    TYPEINFO("experiment/online_learning", "Interactive learning experiment")

  protected:
    Agent *agent_, *test_agent_;
    Environment *environment_;
    State *state_, *curve_;

    size_t runs_, trials_, steps_;
    int test_interval_;
    double rate_;
    std::string output_, load_file_;
    std::string save_every_;


  public:
    OnlineLearningExperiment() : agent_(NULL), test_agent_(NULL), environment_(NULL), state_(NULL), curve_(NULL), runs_(1), trials_(0), steps_(0), test_interval_(-1), rate_(0), save_every_("never")  { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Experiment
    virtual OnlineLearningExperiment *clone() const;
    virtual void run();
};

}

#endif /* GRL_ONLINE_LEARNING_EXPERIMENT_H_ */
