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

#include <grl/representation.h>

#include "tensorflow_api.h"

namespace grl
{
/// Representation using a TensorFlow graph.
class TensorFlowRepresentation : public ParameterizedRepresentation
{
  public:
    TYPEINFO("representation/parameterized/tensorflow", "TensorFlow representation")
    
    typedef std::pair<Vector, Vector> Sample;
    
  protected:
    int inputs_, targets_;
    std::string file_, input_layer_, output_layer_, output_target_, sample_weights_, learning_phase_, init_node_, update_node_;
    std::vector<std::string> outputs_read_, weights_read_, weights_write_, weights_node_;
    mutable std::vector<TF::Shape> weights_shape_;
    
    TF_Graph* graph_;
    TF_Session* session_;
    std::vector<Sample> batch_;
    
    TF_Tensor* batch_input_, *batch_target_;
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
    virtual TensorFlowRepresentation *target()
    {
      return dynamic_cast<TensorFlowRepresentation*>(ParameterizedRepresentation::target());
    }
    
    TF::TensorPtr tensor(TF::Shape shape) const
    {
      return TF::TensorPtr(new TF::Tensor(shape));
    }
    TF::TensorPtr tensor(const Vector &v, TF::Shape shape=TF::Shape()) const
    {
      return TF::TensorPtr(new TF::Tensor(v, shape));
    }
    TF::TensorPtr tensor(const Matrix &m, TF::Shape shape=TF::Shape()) const
    {
      return TF::TensorPtr(new TF::Tensor(m, shape));
    }
    void SessionRun(const std::vector<std::pair<std::string, TF::TensorPtr> > &inputs, const std::vector<std::string> &outputs, const std::vector<std::string> &ops, std::vector<TF::TensorPtr> *results, bool learn=false);
};

}

#endif /* GRL_TENSORFLOW_REPRESENTATION_H_ */
