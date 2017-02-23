/** \file state_machine.h
 * \brief State machine tools.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2016-01-01
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Ivan Koryakovskiy
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
 
#ifndef GRL_STATE_MACHINE_H_
#define GRL_STATE_MACHINE_H_

#include <grl/configurable.h>

namespace grl
{

/// Triggers an event.
class Trigger : public Configurable
{
  public:
    TYPEINFO("trigger", "Event trigger")

  protected:
    Vector min_, max_;
    double delay_, time_begin_;
    bool reset_time_;

  public:
    Trigger() : delay_(0), time_begin_(0), reset_time_(true) { }
    virtual ~Trigger() { }
    virtual Trigger *clone() const
    {
      Trigger *t = new Trigger(*this);
      t->min_ = min_;
      t->max_ = max_;
      t->delay_ = delay_;
      return t;
    }

    // From Configurable
    virtual void request(ConfigurationRequest *config)
    {
      config->push_back(CRP("min", "vector.observation_min", "Minimum of compartment bounding box", min_));
      config->push_back(CRP("max", "vector.observation_max", "Maximum of compartment bounding box", max_));
      config->push_back(CRP("delay", "double", "Settlement delay for which conditions are continuously fullfilled", delay_, CRP::System, 0.0, DBL_MAX));
    }

    virtual void configure(Configuration &config)
    {
      min_ = config["min"].v();
      max_ = config["max"].v();
      delay_ = config["delay"];

      if (min_.size() != max_.size())
        throw bad_param("trigger:{min,max}");
    }

    virtual void reconfigure(const Configuration &config) { }

    // Own
    virtual int check(double time, const Vector &obs)
    {
      if (!min_.size())
        return 1;

      if (obs.size() != min_.size())
        throw bad_param("trigger:{obs,min,max}");

      // check if observation is within a box
      int inside = 1;
      for (size_t ii=0; ii != obs.size(); ++ii)
        if (obs[ii] < min_[ii] || obs[ii] > max_[ii])
          inside = 0;

      // reset time counter if out of box
      if (!inside || reset_time_)
      {
        time_begin_ = time;
        reset_time_ = false;
      }

      // check if conditions are satisfied
      if ((inside) && (time - time_begin_ >= delay_))
      {
        reset_time_ = true; // Time will be reset next time the trigger is questioned
        return 1;
      }

      return 0;
    }

};


}

#endif /* GRL_STATE_MACHINE_H_ */
