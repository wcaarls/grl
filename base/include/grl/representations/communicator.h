  /** \file communicator.h
 * \brief Communicator representation header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-01-24
 *
 * \copyright \verbatim
 * Copyright (c) 2017, Wouter Caarls
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

#ifndef GRL_COMMUNICATOR_REPRESENTATION_H_
#define GRL_COMMUNICATOR_REPRESENTATION_H_

#include <grl/mutex.h>
#include <grl/representation.h>
#include <grl/communicator.h>

namespace grl
{

class CommunicatorRepresentation : public Representation
{
  public:
    TYPEINFO("representation/communicator", "Interface to an out-of-process representation")

    enum MessageType {mtRead, mtWrite, mtUpdate, mtFinalize};

  protected:
    size_t inputs_, outputs_;
    Communicator *communicator_;
    mutable Mutex mutex_;

  public:
    CommunicatorRepresentation() : inputs_(1), outputs_(1), communicator_(NULL) { }

    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Representation
    virtual double read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const;
    virtual void write(const ProjectionPtr projection, const Vector &target, const Vector &alpha);
    virtual void update(const ProjectionPtr projection, const Vector &delta);
    virtual void finalize();
};

}

#endif /* GRL_COMMUNICATOR_REPRESENTATION_H_ */
