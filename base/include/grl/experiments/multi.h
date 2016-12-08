/** \file multi.h
 * \brief Multi experiment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-12-08
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Wouter Caarls
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

#ifndef GRL_MULTI_EXPERIMENT_H_
#define GRL_MULTI_EXPERIMENT_H_

#include <grl/agent.h>
#include <grl/environment.h>
#include <grl/experiment.h>
#include <itc/itc.h>

namespace grl
{

class MultiExperimentInstance: public itc::Thread
{
  protected:
    Experiment *experiment_;
    
  public:
    MultiExperimentInstance(Experiment *experiment) : experiment_(experiment) {}
  
  protected:
    // From itc::Thread
    virtual void run()
    {
      experiment_->run();
      safe_delete(&experiment_);
    }
};

/// Run multiple experiments in parallel.
class MultiExperiment : public Experiment
{
  public:
    TYPEINFO("experiment/multi", "Run multiple experiments in parallel")

  protected:
    int instances_;
    Experiment *prototype_;
    std::vector<MultiExperimentInstance*> experiments_;

  public:
    MultiExperiment() : instances_(4), prototype_(NULL) { }
    ~MultiExperiment()
    {
      for (size_t ii=0; ii < instances_; ++ii)
        safe_delete(&experiments_[ii]);
      experiments_.clear();
    }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Experiment
    virtual void run();
};

}

#endif /* GRL_MULTI_EXPERIMENT_H_ */
