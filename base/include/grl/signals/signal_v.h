/** \file signal_v.h
 * \brief Vector-based signal header definition.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2016-10-11
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

#ifndef GRL_SIGNAL_V_H_
#define GRL_SIGNAL_V_H_

#include <grl/configurable.h>
#include <grl/grl.h>
#include <grl/signal.h>

namespace grl
{
/// Sends signals across different parts of GRL.
class SignalV : public Signal//<Vector>
{
  public:
    TYPEINFO("signal/v", "Vector-based signal")

  public:
    ~SignalV() { }
    virtual SignalV *clone() const
    {
      return new SignalV(*this);
      //return NULL;
    }

    virtual void set(const Vector &in)
    {
      signal_ = in;
    }

    virtual void get(Vector *out) const
    {
      *out = signal_;
    }

  private:
      Vector signal_;
};

}

#endif /* GRL_SIGNAL_V_H_ */
