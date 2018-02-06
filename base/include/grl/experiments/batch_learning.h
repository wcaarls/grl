/** \file batch_learning.h
 * \brief Batch learning experiment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-04-29
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

#ifndef GRL_BATCH_LEARNING_EXPERIMENT_H_
#define GRL_BATCH_LEARNING_EXPERIMENT_H_

#include <grl/environment.h>
#include <grl/predictor.h>
#include <grl/agent.h>
#include <grl/experiment.h>

namespace grl
{

/// Batch learning experiment using randomly sampled experience.
class BatchLearningExperiment : public Experiment
{
  public:
    TYPEINFO("experiment/batch_learning", "Batch learning experiment using randomly sampled experience")

  protected:
    Model *model_;
    Task *task_;
    Predictor *predictor_;
    Agent *test_agent_;
    VectorSignal *state_;

    size_t runs_, batches_, batch_size_;
    double rate_;
    std::string output_;
    
    Vector observation_min_, observation_max_, action_min_, action_max_;

  public:
    BatchLearningExperiment() : model_(NULL), task_(NULL), predictor_(NULL), test_agent_(NULL), state_(NULL), runs_(1), batches_(0), batch_size_(100), rate_(0) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Experiment
    virtual LargeVector run();
};

}

#endif /* GRL_BATCH_LEARNING_EXPERIMENT_H_ */
