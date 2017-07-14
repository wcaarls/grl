/** \file ann.h
 * \brief Artificial neural network representation header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-06-26
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Wouter Caarls
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

struct ANNLayer
{
  size_t size; // Neurons in this layer (excluding bias)
  Eigen::Map<Matrix> W; // weights from previous layer (neurons_prev+1*neurons)
  Matrix delta; // dE/dnet (neurons*samples)
  Matrix activation; // from feedforward pass (neurons*samples)
  Matrix Delta; // weight adjustment (neurons_prev+1*neurons)
  
  // For RPROP
  Matrix eta, prev_Delta;
  
  ANNLayer() : W(NULL, 0, 0) { }
};

/// Artificial neural network.
class ANNRepresentation : public ParameterizedRepresentation
{
  public:
    TYPEINFO("representation/parameterized/ann", "Parameterized artificial neural network representation")

  protected:
    size_t inputs_, outputs_;
    Vector hiddens_;
    double eta_;
    
    LargeVector params_;
    std::vector<ANNLayer> layers_;

    double error_;
    size_t samples_;

  public:
    ANNRepresentation() : inputs_(1), outputs_(1), eta_(0.7), error_(0), samples_(0)
    {
      hiddens_ = VectorConstructor(5);
    }
    
    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    virtual ANNRepresentation &copy(const Configurable &obj);
  
    // From ParameterizedRepresentation
    virtual double read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const;
    virtual void write(const ProjectionPtr projection, const Vector &target, const Vector &alpha);
    virtual void update(const ProjectionPtr projection, const Vector &delta);
    virtual void finalize();

    virtual size_t size() const
    {
      return params_.size();
    }
    
    virtual const LargeVector &params() const
    {
      return params_;
    }
    
    virtual void setParams(const LargeVector &params)
    {
      if (params.size() != params_.size())
      {
        ERROR("Parameter vector size mismatch");
        return;
      }
      
      memcpy(params_.data(), params.data(), params_.size()*sizeof(double));
    }

  protected:
    /// Sigmoid activation
    inline static Matrix activate(const Matrix &x)
    {
      return (1.+(-x.array()).exp()).inverse();
    }
    
    /// Derivative of sigmoid activation
    inline static Matrix dactivate(const Matrix &x)
    {
      return x.array()*(1.-x.array());
    }
    
    void backprop(const Matrix &in, const Matrix &out);
};

}

#endif /* GRL_ANN_REPRESENTATION_H_ */
