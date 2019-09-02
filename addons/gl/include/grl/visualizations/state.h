/** \file state.h
 * \brief Simple state plot header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-15
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

#ifndef GRL_STATE_VISUALIZATION_H_
#define GRL_STATE_VISUALIZATION_H_

#include <string.h>
#include <pthread.h>
#include <deque>

#include <grl/signal.h>
#include <grl/visualization.h>
#include <grl/exporter.h>

namespace grl
{

/// State plot.
class StateVisualization : public Visualization, public itc::Thread
{
  public:
    TYPEINFO("visualization/state", "Plots state values")

  protected:
    VectorSignal *state_;
    Exporter *exporter_;
    std::deque<Vector> points_;
    Vector dims_, min_, max_;
    size_t memory_;
    
    Mutex mutex_;
    bool updated_;
    unsigned int list_;
  
  public:
    StateVisualization() : state_(NULL), exporter_(NULL), memory_(256), updated_(true), list_(0)
    {
    }
    
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Visualization
    virtual void idle(); 
    virtual void draw(); 
    virtual void reshape(int width, int height);
    
    // From itc::Thread
    virtual void run(); 
};

}

#endif /* GRL_STATE_VISUALIZATION_H_ */
