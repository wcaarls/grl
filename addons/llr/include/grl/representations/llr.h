/** \file llr.h
 * \brief Locally weighted regression representation header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
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

#ifndef GRL_LLR_REPRESENTATION_H_
#define GRL_LLR_REPRESENTATION_H_

#include <eigen3/Eigen/Dense>

#include <grl/projector.h>
#include <grl/representation.h>

namespace grl
{

/// Locally linear regression.
class LLRRepresentation : public Representation
{
  public:
    TYPEINFO("representation/llr", "Performs locally linear regression through samples")
    
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Matrix;
    typedef Eigen::Matrix<double, 1, Eigen::Dynamic>              RowVector;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1>              ColumnVector;
    
  protected:
    SampleProjector *projector_;
    double ridge_regression_factor_;
    size_t outputs_, order_;
    Vector min_, max_, input_nominals_, output_nominals_;

  public:
    LLRRepresentation() : projector_(NULL), ridge_regression_factor_(0.00001), outputs_(1), order_(1) { }
  
    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Representation
    virtual LLRRepresentation *clone() const;
    virtual double read(const ProjectionPtr &projection, Vector *result) const ;
    virtual void write(const ProjectionPtr projection, const Vector &target, const Vector &alpha);
    virtual void update(const ProjectionPtr projection, const Vector &delta);
    virtual void finalize();
};

}

#endif /* GRL_LLR_REPRESENTATION_H_ */
