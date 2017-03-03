/** \file communicator.cpp
 * \brief Communicator representation source file.
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

#include <grl/representations/communicator.h>

using namespace grl;

REGISTER_CONFIGURABLE(CommunicatorRepresentation)

void CommunicatorRepresentation::request(const std::string &role, ConfigurationRequest *config)
{
  if (role == "action")
  {
    config->push_back(CRP("inputs", "int.observation_dims", "Number of input dimensions", (int)inputs_, CRP::System, 1));
    config->push_back(CRP("outputs", "int.action_dims", "Number of output dimensions", (int)outputs_, CRP::System, 1));
  }
  else
  {
    if (role == "transition" || role == "value/action")
      config->push_back(CRP("inputs", "int.observation_dims+int.action_dims", "Number of input dimensions", (int)inputs_, CRP::System, 1));
    else if (role == "value/state")
      config->push_back(CRP("inputs", "int.observation_dims", "Number of input dimensions", (int)inputs_, CRP::System, 1));
    else
      config->push_back(CRP("inputs", "Number of input dimensions", (int)inputs_, CRP::System, 1));
      
    if (role == "transition")
      config->push_back(CRP("outputs", "int.observation_dims+2", "Number of output dimensions", (int)outputs_, CRP::System, 1));
    else
      config->push_back(CRP("outputs", "Number of output dimensions", (int)outputs_, CRP::System, 1));
  }
  
  config->push_back(CRP("communicator", "communicator", "Communicator which exchanges messages with the out-of-process representation", communicator_));
}

void CommunicatorRepresentation::configure(Configuration &config)
{
  communicator_ = (Communicator*)config["communicator"].ptr();
  inputs_ = config["inputs"];
  outputs_ = config["outputs"];
  
  Configuration comcfg_in, comcfg_out;
  
  comcfg_in.set("inputs", inputs_);
  comcfg_in.set("outputs", outputs_);
  
  communicator_->setup(comcfg_in, &comcfg_out);
}

void CommunicatorRepresentation::reconfigure(const Configuration &config)
{
}

double CommunicatorRepresentation::read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const
{
  Guard guard(mutex_);

  IndexProjection *ip = dynamic_cast<IndexProjection*>(projection.get());
  if (ip)
  {
    if (ip->indices.size() != 1)
      throw Exception("Communicator representation does not support multi-index projections");
      
    //CRAWL("Reading " << ip->indices[0]);
    communicator_->send(VectorConstructor(mtRead, (double)ip->indices[0]));
  }
  else
  {
    VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
    if (vp)
    {
      if (vp->vector.size() != inputs_)
      {
        ERROR("Projection size " << vp->vector.size() << " doesn't match input size " << inputs_);
        throw bad_param("representation/communicator:inputs");
      }
    
      //CRAWL("Reading " << vp->vector);
      communicator_->send(extend(VectorConstructor(mtRead), vp->vector));
    }
    else
      throw Exception("Communicator representation only supports index or vector projections");
  }
  
  result->resize(outputs_);
  communicator_->recv(result);
  //CRAWL("Result " << *result);

  if (stddev)
    *stddev = Vector();
    
  return (*result)[0];
}

void CommunicatorRepresentation::write(const ProjectionPtr projection, const Vector &target, const Vector &alpha)
{
  Guard guard(mutex_);

  if (target.size() != outputs_)
    throw bad_param("representation/communicator:outputs");

  if (target.size() != alpha.size())
    throw Exception("Learning rate vector does not match target vector");

  IndexProjection *ip = dynamic_cast<IndexProjection*>(projection.get());
  if (ip)
  {
    if (ip->indices.size() != 1)
      throw Exception("Communicator representation does not support multi-index projections");
      
    Vector v(2 + target.size() + alpha.size());
    v << mtWrite, ip->indices[0], target, alpha;

    //CRAWL("Writing " << ip->indices[0] << " -" << alpha << "> " << target);
    communicator_->send(v);
  }
  else
  {
    VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
    if (vp)
    {
      if (vp->vector.size() != inputs_)
      {
        ERROR("Projection size " << vp->vector.size() << " doesn't match input size " << inputs_);
        throw bad_param("representation/communicator:inputs");
      }
        
      Vector v(1 + vp->vector.size() + target.size() + alpha.size());
      v << mtWrite, vp->vector, target, alpha;
    
      //CRAWL("Writing " << vp->vector << " -" << alpha << "> " << target);
      communicator_->send(v);
    }
    else
      throw Exception("Communicator representation only supports index or vector projections");
  }

  Vector _;
  communicator_->recv(&_);
}

void CommunicatorRepresentation::update(const ProjectionPtr projection, const Vector &delta)
{
  Guard guard(mutex_);

  if (delta.size() != outputs_)
    throw bad_param("representation/communicator:outputs");

  IndexProjection *ip = dynamic_cast<IndexProjection*>(projection.get());
  if (ip)
  {
    if (ip->indices.size() != 1)
      throw Exception("Communicator representation does not support multi-index projections");
      
    //CRAWL("Updating " << ip->indices[0] << " + " << delta);
    communicator_->send(extend(VectorConstructor(mtUpdate, (double)ip->indices[0]), delta));
  }
  else
  {
    VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
    if (vp)
    {
      if (vp->vector.size() != inputs_)
      {
        ERROR("Projection size " << vp->vector.size() << " doesn't match input size " << inputs_);
        throw bad_param("representation/communicator:inputs");
      }
        
      Vector v(1 + vp->vector.size() + delta.size());
      v << mtUpdate, vp->vector, delta;
    
      //CRAWL("Updating " << vp->vector << " + " << delta);
      communicator_->send(v);
    }
    else
      throw Exception("Communicator representation only supports index or vector projections");
  }

  Vector _;
  communicator_->recv(&_);
}

void CommunicatorRepresentation::finalize()
{
  Guard guard(mutex_);

  Vector _;

  //CRAWL("Finalizing");
  communicator_->send(VectorConstructor(mtFinalize));
  communicator_->recv(&_);
}
