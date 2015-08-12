/** \file windy.h
 * \brief Windy gridworld environment definitions.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-12
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

#ifndef GRL_WINDY_ENVIRONMENT_H_
#define GRL_WINDY_ENVIRONMENT_H_

#include <grl/environment.h>

namespace grl
{

class WindyGridworldModel : public Model
{
  public:
    enum StateDimIndex { siX, siY, siTime };

    TYPEINFO("model/windy", "Sutton & Barto's windy gridworld model");
    
  public:
    int wind_[10];

  public:
    WindyGridworldModel()
    {
      wind_[0] = wind_[1] = wind_[2] = wind_[9] = 0;
      wind_[3] = wind_[4] = wind_[5] = wind_[8] = 1;
      wind_[6] = wind_[7] = 2;
    }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Model
    virtual WindyGridworldModel *clone() const;
    virtual double step(const Vector &state, const Vector &action, Vector *next) const;
};

/// Windy gridworld movement task.
class WindyGridworldMovementTask : public Task
{
  public:
    TYPEINFO("task/windy/movement", "Windy gridworld movement task")

  public:
    WindyGridworldMovementTask() { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual WindyGridworldMovementTask *clone() const;
    virtual void start(int test, Vector *state) const;
    virtual void observe(const Vector &state, Vector *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const;
    virtual bool invert(const Vector &obs, Vector *state) const;

  protected:
    bool succeeded(const Vector &state) const;
};

}

#endif /* GRL_WINDY_ENVIRONMENT_H_ */
