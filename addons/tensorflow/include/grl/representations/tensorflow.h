/** \file tensorflow.h
 * \brief Tensorflow representation header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-01-31
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

#ifndef GRL_TENSORFLOW_REPRESENTATION_H_
#define GRL_TENSORFLOW_REPRESENTATION_H_

#include <list>
#include <tensorflow/c/c_api.h>

#include <grl/representation.h>

namespace grl
{

/// Average of feature activations.
class TensorFlowRepresentation : public ParameterizedRepresentation
{
  public:
    TYPEINFO("representation/parameterized/tensorflow", "TensorFlow representation")
    
    typedef std::pair<Vector, Vector> Sample;
    class Shape
    {
      protected:
        std::vector<int64_t> dims_;
        
      public:
        Shape(std::vector<int64_t> dims) : dims_(dims) { }
        Shape(TF_Tensor* tensor)
        {
          for (size_t ii=0; ii != TF_NumDims(tensor); ++ii)
            dims_.push_back(TF_Dim(tensor, ii));
        }
        
        const int64_t *dims() const { return dims_.data(); }
        int num_dims() const { return dims_.size(); }
        
        size_t size() const
        {
          size_t sz=1;
          for (size_t ii=0; ii != dims_.size(); ++ii)
            sz *= dims_[ii];
            
          return sz;
        }
    };
    
  protected:
    int inputs_, targets_;
    std::string file_, input_layer_, output_layer_, output_target_, sample_weights_, learning_phase_, init_node_, update_node_;
    std::vector<std::string> outputs_read_, weights_read_, weights_write_, weights_node_;
    mutable std::vector<Shape> weights_shape_;
    
    TF_Graph* graph_;
    TF_Session* session_;
    std::vector<Sample> batch_;
    
    TF_Tensor* input_, *target_;
    size_t counter_;
    
    mutable LargeVector params_;

  public:
    TensorFlowRepresentation() : inputs_(1), targets_(1), counter_(0) { }
  
    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    virtual TensorFlowRepresentation &copy(const Configurable &obj);
  
    // From Representation
    virtual double read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const;
    virtual void write(const ProjectionPtr projection, const Vector &target, const Vector &alpha);
    virtual void update(const ProjectionPtr projection, const Vector &delta);
    virtual void finalize();

    virtual void batchRead(size_t sz);
    virtual void batchWrite(size_t sz);
    virtual void enqueue(const ProjectionPtr &projection);
    virtual void enqueue(const ProjectionPtr &projection, const Vector &target);
    virtual void read(Matrix *out);
    virtual void write();
    
    // From ParameterizedRepresentation
    virtual size_t size() const;
    virtual const LargeVector &params() const;
    virtual void setParams(const LargeVector &params);
};

}

#endif /* GRL_TENSORFLOW_REPRESENTATION_H_ */
