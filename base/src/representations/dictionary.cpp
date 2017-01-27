/** \file dictionary.cpp
 * \brief Dictionary representation source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-01-27
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

#include <grl/representations/dictionary.h>

using namespace grl;

REGISTER_CONFIGURABLE(DictionaryRepresentation)

void DictionaryRepresentation::request(const std::string &role, ConfigurationRequest *config)
{
}

void DictionaryRepresentation::configure(Configuration &config)
{
}

void DictionaryRepresentation::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
    dict_.clear();
}

DictionaryRepresentation &DictionaryRepresentation::copy(const Configurable &obj)
{
  const DictionaryRepresentation &dr = dynamic_cast<const DictionaryRepresentation&>(obj);
  
  dict_ = dr.dict_;

  return *this;
}

double DictionaryRepresentation::read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const
{
  Projection &p = *projection;
  Vector v;
  
  IndexProjection *ip = dynamic_cast<IndexProjection*>(&p);
  if (ip)
  {
    if (ip->indices.size() > 1)
      throw bad_param("representation/dictionary is undefined for multi-index projections");
      
    Vector v = VectorConstructor(ip->indices[0]);
    try {
      *result = dict_.at(v);
    } catch (...) {
      *result = Vector();
    }
  }
  else
  {
    VectorProjection *vp = dynamic_cast<VectorProjection*>(&p);
    if (vp)
    {
      try {
        *result = dict_.at(vp->vector);
      } catch (...) {
        *result = Vector();
      }
    }
    else
      throw Exception("representation/dictionary requires a projector returning IndexProjection or VectorProjection");
  }
  
  if (stddev)
    *stddev = Vector();
  
  if (result->size())
    return (*result)[0];
  else
    return 0;
}

void DictionaryRepresentation::write(const ProjectionPtr projection, const Vector &target, const Vector &alpha)
{
  if (target.size() != alpha.size())
    throw Exception("Learning rate vector does not match target vector");

  Vector value, delta;
  read(projection, &value, NULL);
  
  if (value.size())
    delta = alpha*(target-value);
  else
    delta = alpha*target;
  
  update(projection, delta);
}

void DictionaryRepresentation::update(const ProjectionPtr projection, const Vector &delta)
{
  Projection &p = *projection;
  Vector v;
  
  IndexProjection *ip = dynamic_cast<IndexProjection*>(&p);
  if (ip)
  {
    if (ip->indices.size() > 1)
      throw bad_param("representation/dictionary is undefined for multi-index projections");
      
    v = VectorConstructor(ip->indices[0]);
  }
  else
  {
    VectorProjection *vp = dynamic_cast<VectorProjection*>(&p);
    if (vp)
      v = vp->vector;
    else
      throw Exception("representation/dictionary requires a projector returning IndexProjection or VectorProjection");
  }
  
  if (dict_.count(v))
    dict_[v] += delta;
  else
    dict_[v] = delta;
}
