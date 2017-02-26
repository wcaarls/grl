/** \file dictionary.h
 * \brief Dictionary representation header file.
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

#ifndef GRL_DICTIONARY_REPRESENTATION_H_
#define GRL_DICTIONARY_REPRESENTATION_H_

#include <map>
#include <grl/representation.h>

namespace grl
{

/// Average of feature activations.
class DictionaryRepresentation : public Representation
{
  public:
    TYPEINFO("representation/dictionary", "Stores examples as key-value pairs in a dictionary")
    
  protected:
    std::map<Vector, Vector, std::function<bool(const Vector&,const Vector&)> > dict_;

  public:
    DictionaryRepresentation() : dict_(vless)
    {
    }
  
    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    virtual DictionaryRepresentation &copy(const Configurable &obj);
  
    // From Representation
    virtual double read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const;
    virtual void write(const ProjectionPtr projection, const Vector &target, const Vector &alpha);
    virtual void update(const ProjectionPtr projection, const Vector &delta);
    
  protected:
    static bool vless(const Vector &a, const Vector &b)
    {
      return std::lexicographical_compare(a.data(),a.data()+a.size(),
                                          b.data(),b.data()+b.size());
    }
};

}

#endif /* GRL_DICTIONARY_REPRESENTATION_H_ */
