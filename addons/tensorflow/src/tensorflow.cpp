/** \file tensorflow.cpp
 * \brief Tensorflow representation source file.
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

#include <unistd.h>

#include <vector>

#include <grl/representations/tensorflow.h>
#include "graph.pb.h"

using namespace grl;
using namespace TF;

REGISTER_CONFIGURABLE(TensorFlowRepresentation)

Shape::Shape(TF_Tensor* tensor)
{
  for (size_t ii=0; ii != TF::NumDims(tensor); ++ii)
    dims_.push_back(TF::Dim(tensor, ii));
}

Tensor::Tensor(TF_Tensor* tensor)
{
  tensor_ = tensor;
  data_ = (float*)TF::TensorData(tensor_);
  if (TF::NumDims(tensor_) > 1)
    stride_ = TF::Dim(tensor_, TF::NumDims(tensor_)-1);
  else
    stride_ = 0;
}

Tensor::Tensor(const Shape &shape)
{
  tensor_ = TF::AllocateTensor(TF_FLOAT, shape.dims(), shape.num_dims(), shape.size() * sizeof(float));
  data_ = (float*)TF::TensorData(tensor_);
  if (shape.num_dims() > 1)
    stride_ = shape.dims()[shape.num_dims()-1];
  else
    stride_ = 0;
}

Tensor::Tensor(const Vector &v, Shape shape)
{
  if (!shape.size())
    shape = Shape(v);
    
  if (shape.size() != v.size())
    throw Exception("Requested tensor shape does not match vector size");
    
  tensor_ = TF::AllocateTensor(TF_FLOAT, shape.dims(), shape.num_dims(), shape.size() * sizeof(float));
  float *data = (float*)TF::TensorData(tensor_);
  for (size_t ii=0; ii < shape.size(); ++ii)
    data[ii] = v[ii];
    
  data_ = (float*)TF::TensorData(tensor_);
  if (shape.num_dims() > 1)
    stride_ = shape.dims()[shape.num_dims()-1];
  else
    stride_ = 0;
}

Tensor::Tensor(const Matrix &m, Shape shape)
{
  if (!shape.size())
    shape = Shape(m);
    
  if (shape.size() != m.size())
    throw Exception("Requested tensor shape does not match matrix size");
    
  tensor_ = TF::AllocateTensor(TF_FLOAT, shape.dims(), shape.num_dims(), shape.size() * sizeof(float));
  float *data = (float*)TF::TensorData(tensor_);
  for (size_t ii=0; ii < m.rows(); ++ii)
    for (size_t jj=0; jj < m.cols(); ++jj)
      data[ii*m.cols()+jj] = m(ii, jj);

  data_ = (float*)TF::TensorData(tensor_);
  if (shape.num_dims() > 1)
    stride_ = shape.dims()[shape.num_dims()-1];
  else
    stride_ = 0;
}

Tensor::~Tensor()
{
  if (tensor_)
    TF::DeleteTensor(tensor_);
}

Tensor::operator Vector()
{
  size_t sz = shape().size();
  Vector v = Vector(sz);
  
  float *data = (float*)TF::TensorData(tensor_);
  for (size_t ii=0; ii < sz; ++ii)
    v[ii] = data[ii];
    
  return v;
}

Tensor::operator Matrix()
{
  Shape s = Shape();
  
  if (s.num_dims() != 2)
    throw Exception("Tried to read non-matrix tensor as matrix");
  
  Matrix m = Matrix(s.dims()[0], s.dims()[1]);
  float *data = (float*)TF::TensorData(tensor_);
  for (size_t ii=0; ii < m.rows(); ++ii)
    for (size_t jj=0; jj < m.cols(); ++jj)
      m(ii, jj) = data[ii+m.cols()*jj];
      
  return m;
}

float *Tensor::data()
{
  return (float*) TF::TensorData(tensor_);
}

void TensorFlowRepresentation::request(const std::string &role, ConfigurationRequest *config)
{
  ParameterizedRepresentation::request(role, config);

  config->push_back(CRP("file", "TensorFlow graph stored in binary protocol buffer, or Python command to generate it", file_));
  
  config->push_back(CRP("input_layer", "Name of input layer placeholders (default: inputs of 'inputs' node)", input_layer_));
  config->push_back(CRP("output_layer", "Name of output layer tensors (default: inputs of 'outputs' node)", output_layer_));
  config->push_back(CRP("output_target", "Name of output layer target placeholders (default: nodes containing 'target')", output_target_));
  config->push_back(CRP("sample_weights", "Name of sample weights placeholder (default: nodes containing 'sample_weights')", sample_weights_));
  config->push_back(CRP("learning_phase", "Name of learning phase placeholder (default: nodes containing 'learning_phase')", learning_phase_));

  config->push_back(CRP("init_node", "Name of node to run to initialize weights", init_node_));
  config->push_back(CRP("update_node", "Name of node to run to update weights", update_node_));
}

void TensorFlowRepresentation::configure(Configuration &config)
{
  ParameterizedRepresentation::configure(config);

  file_ = config["file"].str();
  
  input_layer_ = config["input_layer"].str();
  output_layer_ = config["output_layer"].str();
  output_target_ = config["output_target"].str();
  sample_weights_ = config["sample_weights"].str();
  learning_phase_ = config["learning_phase"].str();

  init_node_ = config["init_node"].str();
  update_node_ = config["update_node"].str();
  
  if (file_.empty())
    throw bad_param("representation/tensorflow:file");
    
  NOTICE("Using TensorFlow version " << TF::Version());
  
  std::string graph_pb;
  
  if (file_.find(".py") != std::string::npos)
  {
    NOTICE("Generating TensorFlow model from " << file_);

    char buf[PATH_MAX];
    (void)tmpnam_r(buf);
    std::stringstream ss;
    ss << file_ << " " << buf;
    if (system(ss.str().c_str()))
      throw bad_param("representation/tensorflow:file");
    
    std::ifstream ifs(buf);
    if (!ifs.good())
    {
      ERROR("Error opening generated TensorFlow model " << buf);
      throw bad_param("representation/tensorflow:file");
    }

    ss.str("");
    ss << ifs.rdbuf();
    graph_pb = ss.str();
    unlink(buf);
  }
  else
  {
    NOTICE("Loading TensorFlow model " << file_);
  
    std::ifstream ifs(file_);
    
    if (!ifs.good())
    {
      ERROR("Error opening TensorFlow model " << file_);
      throw bad_param("representation/tensorflow:file");
    }
    
    std::stringstream ss;
    ss << ifs.rdbuf();
    graph_pb = ss.str();
  }
  
  GraphDef graph_def;
  
  if (!graph_def.ParseFromString(graph_pb))
  {
    ERROR("Error parsing ProtoBuf in TensorFlow model " << file_);
    throw bad_param("representation/tensorflow:file");
  }

  bool auto_input = input_layer_.empty(), auto_output = output_layer_.empty();
  
  for (int i = 0; i < graph_def.node_size(); i++)
  {
    auto n = graph_def.node(i);

    if (auto_input && n.name() == "inputs")
      for (int j = 0; j < n.input_size(); j++)
        input_layer_ = input_layer_ + n.input(j).substr(1) + ", ";

    if (auto_output && n.name() == "outputs")
      for (int j = 0; j < n.input_size(); j++)
        output_layer_ = output_layer_ + n.input(j).substr(1) + ", ";

    if (output_target_.empty() && n.name().find("target") != std::string::npos)
      output_target_ = n.name();
      
    if (sample_weights_.empty() && n.name().find("sample_weights") != std::string::npos)
      sample_weights_ = n.name();

    if (learning_phase_.empty() && n.name().find("learning_phase") != std::string::npos)
      learning_phase_ = n.name();

    if (n.op() == "Assign" && n.input_size() == 2 && n.input(1).find("Placeholder") != std::string::npos)
    {
      weights_node_.push_back(n.name());
      weights_read_.push_back(n.input(0) + "/read");
      weights_write_.push_back(n.input(1));
    }
  }
  
  inputs_write_ = cutLongStr(input_layer_);
  outputs_read_ = cutLongStr(output_layer_);

  INFO("Model parameters");
  INFO("  Input layer placeholders  : " << inputs_write_);
  INFO("  Output layer tensors      : " << outputs_read_);
  INFO("  Target placeholder        : " << output_target_);
  INFO("  Sample weights placeholder: " << sample_weights_);
  INFO("  Learning phase placeholder: " << learning_phase_);
  INFO("  Weight tensors            : " << weights_read_);
  INFO("  Weight placeholders       : " << weights_write_);
  INFO("  Weight nodes              : " << weights_node_);
  INFO("  Init node                 : " << init_node_);
  INFO("  Update node               : " << update_node_);

  if (input_layer_.empty())
    throw bad_param("representation/tensorflow:input_layer");

  if (output_layer_.empty())
    throw bad_param("representation/tensorflow:output_layer");

  if (output_target_.empty())
    throw bad_param("representation/tensorflow:target");

  if (update_node_.empty())
    throw bad_param("representation/tensorflow:update_node");

  graph_ = TF::NewGraph();
  TF_Buffer *tf_graph_def = TF::NewBufferFromString((const void*)graph_pb.c_str(), graph_pb.size());
  TF_ImportGraphDefOptions* tf_graph_options = TF::NewImportGraphDefOptions();
  TF_Status *tf_status = TF::NewStatus();
  TF::GraphImportGraphDef(graph_, tf_graph_def, tf_graph_options, tf_status);
  
  if (TF::GetCode(tf_status) != TF_OK)
  {
    ERROR(TF::Message(tf_status));
    throw bad_param("representation/tensorflow:file");
  }
  
  TF_SessionOptions *tf_session_options = TF::NewSessionOptions();
  session_ = TF::NewSession(graph_, tf_session_options, tf_status);
  
  if (TF::GetCode(tf_status) != TF_OK)
  {
    ERROR(TF::Message(tf_status));
    throw bad_param("representation/tensorflow:file");
  }
  
  TF::DeleteBuffer(tf_graph_def);
  TF::DeleteImportGraphDefOptions(tf_graph_options);
  TF::DeleteStatus(tf_status);
  TF::DeleteSessionOptions(tf_session_options);

  input_shapes_ = IndexVector(inputs_write_.size());
  for (size_t ii=0; ii < inputs_write_.size(); ++ii)
    input_shapes_[ii] = GraphGetTensorElements({OperationByName(inputs_write_[ii].c_str()), 0});
  inputs_ = input_shapes_.sum();
  targets_ = GraphGetTensorElements({OperationByName(output_target_.c_str()), 0});
  
  // Initialize weights
  reset();
}

void TensorFlowRepresentation::reconfigure(const Configuration &config)
{
  ParameterizedRepresentation::reconfigure(config);

  if (config.has("action") && config["action"].str() == "reset")
  {
    if (!init_node_.empty())
    {
      INFO("Initializing weights");
      
      // Run session for init node
      TF_Operation *tf_operation = OperationByName(init_node_.c_str()); 
      TF_Status *tf_status = TF::NewStatus();
      TF::SessionRun(session_, 0,
                    0, 0, 0,           // Inputs
                    0, 0, 0,           // Outputs
                    &tf_operation, 1,  // Operations
                    0, tf_status);
                
      if (TF::GetCode(tf_status) != TF_OK)
      {
        ERROR(TF::Message(tf_status));
        throw bad_param("representation/tensorflow:file");
      }
      
      TF::DeleteStatus(tf_status);
    }
    
    batch_.clear();
    synchronize();
    
    // Get parameter vector size
    params();
  }
}

// Not reentrant
TensorFlowRepresentation &TensorFlowRepresentation::copy(const Configurable &obj)
{
  const TensorFlowRepresentation &tfr = dynamic_cast<const TensorFlowRepresentation&>(obj);
  
  setParams(tfr.params());

  return *this;
}

double TensorFlowRepresentation::read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const
{
  timer tmr, sess;

  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  
  if (vp)
  {
    // Prepare SessionRun inputs
    std::vector<TF_Output>  input_ops;
    std::vector<TF_Tensor*> input_val;
    for (size_t ii=0, nn=0; ii < inputs_write_.size() && nn < vp->vector.size(); nn += input_shapes_[ii++])
    {
      input_ops.push_back({OperationByName(inputs_write_[ii].c_str()), 0});

      // Convert input to tensor
      int64_t shape[] = {1, (int)input_shapes_[ii]};
      input_val.push_back(TF::AllocateTensor(TF_FLOAT, shape, 2, input_shapes_[ii] * sizeof(float)));
      
      float *data = (float*)TF::TensorData(input_val[ii]);
      for (size_t jj=0; jj < input_shapes_[ii] && jj+nn < vp->vector.size(); ++jj)
        data[jj] = vp->vector[jj+nn];
    }
      
    if (!learning_phase_.empty())
    {
      // Prepare learning phase tensor
      TF_Tensor *learning = TF::AllocateTensor(TF_BOOL, NULL, 0, sizeof(bool));
      *((bool*)TF::TensorData(learning)) = false;

      input_ops.push_back({OperationByName(learning_phase_.c_str()), 0});
      input_val.push_back(learning);
    }
    
    // Prepare SessionRun outputs
    std::vector<TF_Output>  output_ops(outputs_read_.size());
    std::vector<TF_Tensor*> output_val(outputs_read_.size());
    for (size_t oo=0; oo < outputs_read_.size(); ++oo)
      output_ops[oo] = {OperationByName(outputs_read_[oo].c_str()), 0};
    
    sess.restart();
      
    // Run session
    TF_Status *tf_status = TF::NewStatus();
    TF::SessionRun(session_, 0, 
                  input_ops.data(), input_val.data(), input_ops.size(),
                  output_ops.data(), output_val.data(), output_ops.size(),
                  0, 0,
                  0, tf_status);
                  
    sess.stop();
                  
    if (TF::GetCode(tf_status) != TF_OK)
    {
      ERROR(TF::Message(tf_status));
      throw Exception("Could not run prediction graph");
    }
    
    // Allocate result vector
    size_t num_outputs=0;
    for (size_t oo=0; oo < output_val.size(); ++oo)
      num_outputs += TF::Dim(output_val[oo], 1);
    result->resize(num_outputs);
    
    // Convert output tensor to result vector
    size_t residx=0;
    for (size_t oo=0; oo < output_val.size(); ++oo)
    {
      float *data = (float*)TF::TensorData(output_val[oo]);
      for (size_t ii=0; ii < TF::Dim(output_val[oo], 1); ++ii)
        (*result)[residx++] = data[ii];
    }
    
    // Clean up
    for (size_t ii=0; ii < input_val.size(); ++ii)
      TF::DeleteTensor(input_val[ii]);
    for (size_t oo=0; oo < output_val.size(); ++oo)
      TF::DeleteTensor(output_val[oo]);
    TF::DeleteStatus(tf_status);
  }
  else
    throw Exception("representation/tensorflow requires a projector returning a VectorProjection");
    
  if (stddev) *stddev = Vector();

  TIMING("single read: " << tmr.elapsed() << " total, " << sess.elapsed() << " TF");
  
  return (*result)[0];
}

void TensorFlowRepresentation::write(const ProjectionPtr projection, const Vector &target, const Vector &alpha)
{
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  
  if (vp)
  {
    if (prod(alpha) == 1)
    {
      batch_.push_back({vp->vector, target});
    }
    else
    {
      Vector v;
      read(projection, &v, NULL);
      batch_.push_back({vp->vector, (1-alpha)*v + alpha*target});
    }
  }
  else
    throw Exception("representation/tensorflow requires a projector returning a VectorProjection");
}

void TensorFlowRepresentation::update(const ProjectionPtr projection, const Vector &delta)
{
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  
  if (vp)
  {
    Vector v;
    read(projection, &v, NULL);
    batch_.push_back({vp->vector, v+delta});
  }
  else
    throw Exception("representation/tensorflow requires a projector returning a VectorProjection");
}

void TensorFlowRepresentation::finalize()
{
  timer tmr, sess;

  // Prepare SessionRun inputs
  std::vector<TF_Output>  input_ops;
  std::vector<TF_Tensor*> input_val;
  for (size_t ii=0, nn=0; ii < inputs_write_.size() && nn < batch_[0].first.size(); nn += input_shapes_[ii++])
  {
    input_ops.push_back({OperationByName(inputs_write_[ii].c_str()), 0});

    // Convert input to tensor
    int64_t shape[] = {(int)batch_.size(), (int)input_shapes_[ii]};
    input_val.push_back(TF::AllocateTensor(TF_FLOAT, shape, 2, batch_.size() * input_shapes_[ii] * sizeof(float)));
    
    float *data = (float*)TF::TensorData(input_val[ii]);
    for (size_t jj=0; jj < batch_.size(); ++jj)
      for (size_t kk=0; kk < input_shapes_[ii] && kk+nn < batch_[jj].first.size(); ++kk)
        data[jj*batch_[jj].first.size()+kk] = batch_[jj].first[kk+nn];
  }
  
  input_ops.push_back({OperationByName(output_target_.c_str()), 0});

  // Convert target to tensor
  int64_t target_shape[] = {(int)batch_.size(), (int)batch_[0].second.size()};
  input_val.push_back(TF::AllocateTensor(TF_FLOAT, target_shape, 2, batch_.size() * batch_[0].second.size() * sizeof(float)));
  float *data = (float*)TF::TensorData(input_val[input_val.size()-1]);
  for (size_t ii=0; ii < batch_.size(); ++ii)
    for (size_t jj=0; jj < batch_[ii].second.size(); ++jj)
      data[ii*batch_[ii].second.size()+jj] = batch_[ii].second[jj];
      
  if (!learning_phase_.empty())
  {
    // Prepare learning phase tensor
    TF_Tensor *learning = TF::AllocateTensor(TF_BOOL, NULL, 0, sizeof(bool));
    *((bool*)TF::TensorData(learning)) = true;

    input_ops.push_back({OperationByName(learning_phase_.c_str()), 0});
    input_val.push_back(learning);
  }
  if (!sample_weights_.empty())
  {
    // Prepare sample weight tensor
    TF_Tensor *weights = TF::AllocateTensor(TF_FLOAT, target_shape, 1, batch_.size() * sizeof(float));
    float *data = (float*)TF::TensorData(weights);
    for (size_t ii=0; ii < batch_.size(); ++ii)
      data[ii] = 1.;

    input_ops.push_back({OperationByName(sample_weights_.c_str()), 0});
    input_val.push_back(weights);
  }
  
  sess.restart();
  
  // Run session for update node
  TF_Operation *tf_operation = OperationByName(update_node_.c_str()); 
  TF_Status *tf_status = TF::NewStatus();
  TF::SessionRun(session_, 0, 
                input_ops.data(), input_val.data(), input_ops.size(),
                0, 0, 0,
                &tf_operation, 1,
                0, tf_status);
                
  sess.stop();
                
  if (TF::GetCode(tf_status) != TF_OK)
  {
    ERROR(TF::Message(tf_status));
    throw Exception("Could not run learning graph");
  }

  // Clean up
  for (size_t ii=0; ii < input_val.size(); ++ii)
    TF::DeleteTensor(input_val[ii]);
  TF::DeleteStatus(tf_status);

  checkSynchronize();
  
  TIMING("finalize (" << batch_.size() << "): " << tmr.elapsed() << " total, " << sess.elapsed() << " TF");
  
  batch_.clear();
}

void TensorFlowRepresentation::batchRead(size_t sz)
{
  pthread_mutex_lock(&mutex_);
  
  batch_inputs_ = std::vector<TF_Tensor*>(inputs_write_.size());
  for (size_t ii=0; ii < inputs_write_.size(); ++ii)
  {
    // Allocate input tensor
    int64_t shape[] = {(int)sz, (int)input_shapes_[ii]};
    batch_inputs_[ii] = TF::AllocateTensor(TF_FLOAT, shape, 2, sz * input_shapes_[ii] * sizeof(float));
  }
  
  counter_ = 0;
}

void TensorFlowRepresentation::batchWrite(size_t sz)
{
  pthread_mutex_lock(&mutex_);

  batch_inputs_ = std::vector<TF_Tensor*>(inputs_write_.size());
  for (size_t ii=0; ii < inputs_write_.size(); ++ii)
  {
    // Allocate input tensor
    int64_t shape[] = {(int)sz, (int)input_shapes_[ii]};
    batch_inputs_[ii] = TF::AllocateTensor(TF_FLOAT, shape, 2, sz * input_shapes_[ii] * sizeof(float));
  }
  
  // Allocate target tensor
  int64_t target_shape[] = {(int)sz, (int)targets_};
  batch_target_ = TF::AllocateTensor(TF_FLOAT, target_shape, 2, sz * targets_ * sizeof(float));
  
  counter_ = 0;
}

void TensorFlowRepresentation::enqueue(const ProjectionPtr &projection)
{
//  CRAWL("enqueue read");
  
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  
  if (vp)
  {
    // Convert input to tensor
    size_t ii=0;
    for (size_t nn=0; ii < batch_inputs_.size() && nn < vp->vector.size(); nn += input_shapes_[ii++])
    {
      float *data = (float*)TF::TensorData(batch_inputs_[ii]) + counter_*(int)input_shapes_[ii];
      for (size_t jj=0; jj < input_shapes_[ii] && jj+nn < vp->vector.size(); ++jj)
        data[jj] = vp->vector[jj+nn];
    }
    
    if (ii < batch_inputs_.size())
      batch_inputs_.erase(batch_inputs_.begin()+ii, batch_inputs_.end());
    
    counter_++;
  }
  else
    throw Exception("representation/tensorflow requires a projector returning a VectorProjection");
}

void TensorFlowRepresentation::enqueue(const ProjectionPtr &projection, const Vector &target)
{
//  CRAWL("enqueue write");
  
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  
  if (vp)
  {
    // Convert input to tensor
    size_t ii=0;
    for (size_t nn=0; ii < batch_inputs_.size() && nn < vp->vector.size(); nn += input_shapes_[ii++])
    {
      float *data = (float*)TF::TensorData(batch_inputs_[ii]) + counter_*(int)input_shapes_[ii];
      for (size_t jj=0; jj < input_shapes_[ii] && jj+nn < vp->vector.size(); ++jj)
        data[jj] = vp->vector[jj+nn];
    }

    if (ii < batch_inputs_.size())
      batch_inputs_.erase(batch_inputs_.begin()+ii, batch_inputs_.end());

    // Convert target to tensor
    float *data = (float*)TF::TensorData(batch_target_) + counter_*targets_;
    for (size_t ii=0; ii < targets_; ++ii)
      data[ii] = target[ii];

    counter_++;
  }
  else
    throw Exception("representation/tensorflow requires a projector returning a VectorProjection");
}

void TensorFlowRepresentation::read(Matrix *out)
{
  timer tmr, sess;

  // Prepare SessionRun inputs
  std::vector<TF_Output>  input_ops(batch_inputs_.size());
  std::vector<TF_Tensor*> input_val = batch_inputs_;
  for (size_t ii=0; ii < batch_inputs_.size(); ++ii)
    input_ops[ii] = {OperationByName(inputs_write_[ii].c_str()), 0};

  pthread_mutex_unlock(&mutex_);
    
  if (!learning_phase_.empty())
  {
    // Prepare learning phase tensor
    TF_Tensor *learning = TF::AllocateTensor(TF_BOOL, NULL, 0, sizeof(bool));
    *((bool*)TF::TensorData(learning)) = false;

    input_ops.push_back({OperationByName(learning_phase_.c_str()), 0});
    input_val.push_back(learning);
  }
  
  // Prepare SessionRun outputs
  std::vector<TF_Output>  output_ops(outputs_read_.size());
  for (size_t oo=0; oo < outputs_read_.size(); ++oo)
    output_ops[oo] = {OperationByName(outputs_read_[oo].c_str()), 0};
  std::vector<TF_Tensor*> output_val(outputs_read_.size());
  
  sess.restart();

  // Run session
  TF_Status *tf_status = TF::NewStatus();
  TF::SessionRun(session_, 0, 
                input_ops.data(), input_val.data(), input_ops.size(),
                output_ops.data(), output_val.data(), output_ops.size(),
                0, 0,
                0, tf_status);
                
  sess.stop();
                
  if (TF::GetCode(tf_status) != TF_OK)
  {
    ERROR(TF::Message(tf_status));
    throw Exception("Could not run prediction graph");
  }

  // Allocate result matrix
  size_t num_outputs=0;
  for (size_t oo=0; oo < output_val.size(); ++oo)
    num_outputs += TF::Dim(output_val[oo], 1);
  out->resize(TF::Dim(output_val[0], 0), num_outputs);
  
  // Convert output tensor to result matrix
  size_t residx=0;
  for (size_t oo=0; oo < output_val.size(); ++oo)
  {
    float *data = (float*)TF::TensorData(output_val[oo]);
    for (size_t ii=0; ii < TF::Dim(output_val[oo], 0); ++ii)
      for (size_t jj=0; jj < TF::Dim(output_val[oo], 1); ++jj)
        (*out)(ii, residx+jj) = data[ii*TF::Dim(output_val[oo], 1)+jj];
    residx += TF::Dim(output_val[oo], 1);
  }
  
  // Clean up
  for (size_t ii=0; ii < input_val.size(); ++ii)
    TF::DeleteTensor(input_val[ii]);
  for (size_t oo=0; oo < output_val.size(); ++oo)
    TF::DeleteTensor(output_val[oo]);
  TF::DeleteStatus(tf_status);

  TIMING("batch read (" << out->rows() << "): " << tmr.elapsed() << " total, " << sess.elapsed() << " TF");
}

void TensorFlowRepresentation::write()
{
  timer tmr, sess;

  // Prepare SessionRun inputs
  std::vector<TF_Output>  input_ops(batch_inputs_.size());
  std::vector<TF_Tensor*> input_val = batch_inputs_;
  for (size_t ii=0; ii < batch_inputs_.size(); ++ii)
    input_ops[ii] = {OperationByName(inputs_write_[ii].c_str()), 0};

  input_ops.push_back({OperationByName(output_target_.c_str()), 0});
  input_val.push_back(batch_target_);

  int counter = counter_;
  pthread_mutex_unlock(&mutex_);
                                      
  if (!learning_phase_.empty())
  {
    // Prepare learning phase tensor
    TF_Tensor *learning = TF::AllocateTensor(TF_BOOL, NULL, 0, sizeof(bool));
    *((bool*)TF::TensorData(learning)) = true;

    input_ops.push_back({OperationByName(learning_phase_.c_str()), 0});
    input_val.push_back(learning);
  }
  if (!sample_weights_.empty())
  {
    // Prepare sample weight tensor
    int64_t shape[] = {(int)counter};
    TF_Tensor *weights = TF::AllocateTensor(TF_FLOAT, shape, 1, counter * sizeof(float));
    float *data = (float*)TF::TensorData(weights);
    for (size_t ii=0; ii < counter; ++ii)
      data[ii] = 1.;

    input_ops.push_back({OperationByName(sample_weights_.c_str()), 0});
    input_val.push_back(weights);
  }

  sess.restart();

  // Run session for update node
  TF_Operation *tf_operation = OperationByName(update_node_.c_str()); 
  TF_Status *tf_status = TF::NewStatus();
  TF::SessionRun(session_, 0, 
                input_ops.data(), input_val.data(), input_ops.size(),
                0, 0, 0,
                &tf_operation, 1,
                0, tf_status);

  sess.stop();
                
  if (TF::GetCode(tf_status) != TF_OK)
  {
    ERROR(TF::Message(tf_status));
    throw Exception("Could not run learning graph");
  }

  // Clean up
  for (size_t ii=0; ii < input_val.size(); ++ii)
    TF::DeleteTensor(input_val[ii]);
  TF::DeleteStatus(tf_status);
  
  checkSynchronize();

  TIMING("batch write (" << counter << "): " << tmr.elapsed() << " total, " << sess.elapsed() << " TF");
}

size_t TensorFlowRepresentation::size() const
{
  return params_.size();
}

const LargeVector &TensorFlowRepresentation::params() const
{
  // Prepare SessionRun outputs
  std::vector<TF_Output>  output_ops(weights_read_.size());
  for (size_t oo=0; oo < weights_read_.size(); ++oo)
    output_ops[oo] = {OperationByName(weights_read_[oo].c_str()), 0};
  std::vector<TF_Tensor*> output_val(weights_read_.size());
    
  // Run session
  TF_Status *tf_status = TF::NewStatus();
  TF::SessionRun(session_, 0, 
                0, 0, 0,
                output_ops.data(), output_val.data(), output_ops.size(),
                0, 0,
                0, tf_status);
                
  if (TF::GetCode(tf_status) != TF_OK)
  {
    ERROR(TF::Message(tf_status));
    throw Exception("Could not run weight reading graph");
  }

  // Allocate result vector
  size_t n = 0;
  weights_shape_.clear();
  for (size_t ii=0; ii < output_val.size(); ++ii)
  {
    weights_shape_.push_back(Shape(output_val[ii]));
    n += weights_shape_.back().size();
  }
  
  params_.resize(n);
  
  // Convert output tensor to result vector
  n = 0;
  for (size_t ii=0; ii < output_val.size(); ++ii)
  {
    float *data = (float*)TF::TensorData(output_val[ii]);
    for (size_t jj=0; jj < weights_shape_[ii].size(); ++jj)
      params_[n+jj] = data[jj];
    n += weights_shape_[ii].size();
  }

  // Clean up
  for (size_t oo=0; oo < output_val.size(); ++oo)
    TF::DeleteTensor(output_val[oo]);
  TF::DeleteStatus(tf_status);

  return params_;
}

void TensorFlowRepresentation::setParams(const LargeVector &params)
{
  if (params.size() != params_.size())
  {
    ERROR("Parameter vector size mismatch");
    return;
  }
  
  params_ = params;
  
  // Prepare SessionRun inputs
  std::vector<TF_Output> input_ops;
  std::vector<TF_Tensor*> input_val;
  
  // Convert input to tensor
  size_t n=0;
  for (size_t ii=0; ii < weights_shape_.size(); ++ii)
  {
    TF_Tensor *w = TF::AllocateTensor(TF_FLOAT, weights_shape_[ii].dims(), weights_shape_[ii].num_dims(), weights_shape_[ii].size() * sizeof(float));
    float *data = (float*)TF::TensorData(w);
    
    for (size_t jj=0; jj < weights_shape_[ii].size(); ++jj)
      data[jj] = params_[n+jj];
    n += weights_shape_[ii].size();
    
    input_ops.push_back({OperationByName(weights_write_[ii].c_str()), 0});
    input_val.push_back(w);
  }
                                      
  // Run session for weight nodes
  std::vector<TF_Operation*> operations(weights_node_.size());
  for (size_t ii=0; ii < operations.size(); ++ii)
    operations[ii] = OperationByName(weights_node_[ii].c_str());
  TF_Status *tf_status = TF::NewStatus();
  TF::SessionRun(session_, 0, 
                input_ops.data(), input_val.data(), input_ops.size(),
                0, 0, 0,
                operations.data(), operations.size(),
                0, tf_status);
                
  if (TF::GetCode(tf_status) != TF_OK)
  {
    ERROR(TF::Message(tf_status));
    throw Exception("Could not run weight writing graph");
  }

  // Clean up
  for (size_t ii=0; ii < input_val.size(); ++ii)
    TF::DeleteTensor(input_val[ii]);
  TF::DeleteStatus(tf_status);
}

void TensorFlowRepresentation::SessionRun(const std::vector<std::pair<std::string, TensorPtr> > &inputs, const std::vector<std::string> &outputs, const std::vector<std::string> &ops, std::vector<TensorPtr> *results, bool learn)
{
  timer tmr, sess;

  std::vector<TF_Output> input_ops(inputs.size()), output_ops(outputs.size());
  std::vector<TF_Tensor*> input_val(inputs.size()), output_val(outputs.size()), input_del;
  std::vector<TF_Operation*> operations(ops.size());
  
  for (size_t ii=0; ii != inputs.size(); ++ii)
  {
    input_ops[ii] = {OperationByName(inputs[ii].first.c_str()), 0};
    input_val[ii] = *inputs[ii].second;
  }
  
  if (!learning_phase_.empty())
  {
    // Prepare learning phase tensor
    TF_Tensor *learning = TF::AllocateTensor(TF_BOOL, NULL, 0, sizeof(bool));
    *((bool*)TF::TensorData(learning)) = learn;

    input_ops.push_back({OperationByName(learning_phase_.c_str()), 0});
    input_val.push_back(learning);
    input_del.push_back(learning);
  }
  if (!sample_weights_.empty() && learn)
  {
    // Prepare sample weight tensor
    Shape shape = inputs[0].second->shape();
    TF_Tensor *weights = TF::AllocateTensor(TF_FLOAT, shape.dims(), 1, shape.dims()[0] * sizeof(float));
    float *data = (float*)TF::TensorData(weights);
    for (size_t ii=0; ii < shape.dims()[0]; ++ii)
      data[ii] = 1.;

    input_ops.push_back({OperationByName(sample_weights_.c_str()), 0});
    input_val.push_back(weights);
    input_del.push_back(weights);
  }

  for (size_t ii=0; ii != outputs.size(); ++ii)
    output_ops[ii] = {OperationByName(outputs[ii].c_str()), 0};

  for (size_t ii=0; ii != ops.size(); ++ii)
    operations[ii] = OperationByName(ops[ii].c_str());

  sess.restart();
  
  TF_Status *tf_status = TF::NewStatus();
  TF::SessionRun(session_, 0,
                input_ops.data(), input_val.data(), input_ops.size(),
                output_ops.data(), output_val.data(), output_ops.size(),
                operations.data(), operations.size(),
                0, tf_status);
                
  sess.stop(); 

  if (TF::GetCode(tf_status) != TF_OK)
  {
    ERROR(TF::Message(tf_status));
    
    if (inputs.size())
    {
      ERROR("Inputs");
      for (size_t ii=0; ii != inputs.size(); ++ii)
        ERROR(" - " << inputs[ii].first << ": shape " << inputs[ii].second->shape()); 
    }
    if (outputs.size())
    {
      ERROR("Outputs");
      for (size_t ii=0; ii != outputs.size(); ++ii)
        ERROR(" - " << outputs[ii]);
    }
    if (ops.size())
    {
      ERROR("Operations");
      for (size_t ii=0; ii != ops.size(); ++ii)
        ERROR(" - " << ops[ii]);
    }

    throw Exception("Could not run custom TensorFlow graph");
  }
  
  results->resize(output_val.size());
  for (size_t ii=0; ii != outputs.size(); ++ii)
    (*results)[ii] = TensorPtr(new Tensor(output_val[ii]));
    
  for (size_t ii=0; ii != input_del.size(); ++ii)
    TF::DeleteTensor(input_del[ii]);
  TF::DeleteStatus(tf_status);
  
  if (learn)
    checkSynchronize();
    
  TIMING("custom session: " << tmr.elapsed() << " total, " << sess.elapsed() << " TF");
}

TF_Operation* TensorFlowRepresentation::OperationByName(const char* name) const
{
  TF_Operation *op = TF::GraphOperationByName(graph_, name);
  if (!op)
  {
    ERROR("Could not find operation '" << name << "'");
    throw Exception("Could not run graph");
  }
  
  return op;
}

int64_t TensorFlowRepresentation::GraphGetTensorElements(TF_Output output) const
{
  int64_t shape[2];
  TF_Status *tf_status = TF::NewStatus();

  TF::GraphGetTensorShape(graph_, output, shape, 2, tf_status);
  if (TF::GetCode(tf_status) != TF_OK)
  {
    ERROR(TF::Message(tf_status));
    throw Exception("Could not find tensor shape");
  }
  
  TF::DeleteStatus(tf_status);
  
  return shape[1];
}
