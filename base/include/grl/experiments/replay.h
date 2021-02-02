/** \file replay.h
 * \brief Replay experiment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2021-02-02
 *
 * \copyright \verbatim
 * Copyright (c) 2021, Wouter Caarls
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

#ifndef GRL_REPLAY_EXPERIMENT_H_
#define GRL_REPLAY_EXPERIMENT_H_

#include <grl/signal.h>
#include <grl/importer.h>
#include <grl/experiment.h>

namespace grl
{

/// Experiment that replays a previous run
class ReplayExperiment : public Experiment
{
  public:
    TYPEINFO("experiment/replay", "Experiment that replays a previous run")

  protected:
    Importer *importer_;
  
    double rate_;
    int skip_;
    VectorSignal *state_, *action_, *reward_, *curve_;

  public:
    ReplayExperiment() : importer_(NULL), rate_(0.), skip_(0) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Experiment
    virtual LargeVector run();
};

}

#endif /* GRL_REPLAY_EXPERIMENT_H_ */
