/** \file communicator.h
 * \brief Communicator header file.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@gmail.com>
 * \date      2016-02-09
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

#ifndef GRL_COMMUNICATOR_H_
#define GRL_COMMUNICATOR_H_

#include <grl/configurable.h>

namespace grl
{

/// Base communicator class
class Communicator: public Configurable
{
public:
  virtual ~Communicator() { }

  /// Exchange metadata.
  virtual void setup(const Configuration &in, Configuration *out) { }

  /// Send data.
  virtual void send(const Vector &v) const = 0;

  /// Receive data.
  virtual bool recv(Vector *v) const = 0;
};

}

#endif /* GRL_COMMUNICATOR_H_ */
