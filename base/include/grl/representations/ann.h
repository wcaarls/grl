/** \file ann.h
 * \brief Artificial neural network representation header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-14
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

#ifndef GRL_ANN_REPRESENTATION_H_
#define GRL_ANN_REPRESENTATION_H_

#include <grl/representation.h>

namespace grl
{

/// Artificial neural network.
class ANNRepresentation : public ParameterizedRepresentation
{
  public:
    TYPEINFO("representation/parameterized/ann")

  protected:
    Vector input_min_, input_max_, output_min_, output_max_;
    size_t inputs_, hiddens_, outputs_;
    Vector weights_, state_;
    double steepness_;
    int bias_, recurrent_;

  public:
    ANNRepresentation() : inputs_(0), hiddens_(10), outputs_(0), steepness_(5), bias_(1), recurrent_(0) { }
    
    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From ParameterizedRepresentation
    virtual ANNRepresentation *clone() const;
    virtual double read(const ProjectionPtr &projection, Vector *result) const ;
    virtual void write(const ProjectionPtr projection, const Vector &target, const Vector &alpha);
    virtual void update(const ProjectionPtr projection, const Vector &delta);

    virtual size_t size() const
    {
      return (inputs_+bias_+recurrent_)*hiddens_+(hiddens_+bias_)*outputs_;
    }
    
    virtual const Vector &params() const
    {
      return weights_;
    }
    
    virtual Vector &params()
    {
      return weights_;
    }

  protected:
    inline double activate(double x) const
    {
      return 1/(1+exp(-steepness_*x));
    }
};

}

#endif /* GRL_ANN_REPRESENTATION_H_ */
