/** \file state.h
 * \brief Transferrable state object.
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

#ifndef GRL_STATE_H_
#define GRL_STATE_H_

#include <grl/configurable.h>
#include <grl/mutex.h>

namespace grl
{

/// Encapsulates a system state.
class State : public Configurable
{
  public:
    TYPEINFO("state");
    
  protected:
    mutable Mutex mutex_;
    Vector state_;

  public:
    virtual State *clone() const
    {
      return new State(*this);
    }
    
    virtual const Vector &get() const
    {
      Guard guard(mutex_);
      return state_;
    }
    
    virtual void set(const Vector &state)
    {
      Guard guard(mutex_);
      state_ = state;
    }
};

}

#endif /* GRL_STATE_H_ */
