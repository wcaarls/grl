/** \file signal.h
 * \brief Generic signal definition.
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

#ifndef GRL_SIGNAL_H_
#define GRL_SIGNAL_H_

#include <grl/configurable.h>
#include <grl/grl.h>

namespace grl
{
/*
/// Maps states to actions.
template <class T>
class Signal : public Configurable
{
  public:
    virtual ~Signal() { }
    virtual Signal *clone() const = 0;
    
    virtual void set(const T &in) = 0;
    virtual void get(T *out) const = 0;
};
*/

/// Maps states to actions.
class Signal : public Configurable
{
  public:
    virtual ~Signal() { }
    virtual Signal *clone() const = 0;

    virtual void set(const Vector &in) = 0;
    virtual void get(Vector *out) const = 0;
};

}

#endif /* GRL_SIGNAL_H_ */
