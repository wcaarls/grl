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

#include <vector>

#include <grl/representations/tensorflow.h>
#include "graph.pb.h"

using namespace grl;

REGISTER_CONFIGURABLE(TensorFlowRepresentation)

void TensorFlowRepresentation::request(const std::string &role, ConfigurationRequest *config)
{
  if (role == "action")
  {
    config->push_back(CRP("inputs", "int.observation_dims", "Number of input dimensions", (int)inputs_, CRP::System, 1));
    config->push_back(CRP("targets", "int.action_dims", "Number of target dimensions", (int)targets_, CRP::System, 1));
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
      config->push_back(CRP("targets", "int.observation_dims+2", "Number of target dimensions", (int)targets_, CRP::System, 1));
    else
      config->push_back(CRP("targets", "Number of target dimensions", (int)targets_, CRP::System, 1));
  }
  
  config->push_back(CRP("file", "TensorFlow graph stored in binary protocol buffer", file_));
  
  config->push_back(CRP("input_layer", "Name of input layer placeholders", input_layer_));
  config->push_back(CRP("output_layer", "Name of output layer tensors", output_layer_));
  config->push_back(CRP("target", "Name of output layer target placeholders", output_target_));
  config->push_back(CRP("sample_weights", "Name of sample weights placeholder", sample_weights_));
  config->push_back(CRP("learning_phase", "Name of learning phase placeholder", learning_phase_));

  config->push_back(CRP("init_node", "Name of node to run to initialize weights", init_node_));
  config->push_back(CRP("update_node", "Name of node to run to update weights", update_node_));
}

void TensorFlowRepresentation::configure(Configuration &config)
{
  inputs_ = config["inputs"];
  targets_ = config["targets"];
  file_ = config["file"].str();
  
  input_layer_ = config["input_layer"].str();
  output_layer_ = config["output_layer"].str();
  output_target_ = config["target"].str();
  sample_weights_ = config["sample_weights"].str();
  learning_phase_ = config["learning_phase"].str();

  init_node_ = config["init_node"].str();
  update_node_ = config["update_node"].str();
  
  if (file_.empty())
    throw bad_param("representation/tensorflow:file");
    
  NOTICE("Using TensorFlow version " << TF_Version());
  
  NOTICE("Loading TensorFlow model " << file_);
  
  std::ifstream ifs(file_);
  
  if (!ifs.good())
  {
    ERROR("Error opening TensorFlow model " << file_);
    throw bad_param("representation/tensorflow:file");
  }
  
  std::stringstream ss;
  ss << ifs.rdbuf();
  
  GraphDef graph_def;
  
  if (!graph_def.ParseFromString(ss.str()))
  {
    ERROR("Error parsing ProtoBuf in TensorFlow model " << file_);
    throw bad_param("representation/tensorflow:file");
  }
  
  bool auto_output = output_layer_.empty();

  for (int i = 0; i < graph_def.node_size(); i++)
  {
    auto n = graph_def.node(i);
    
    if (input_layer_.empty() && n.name().find("input") != std::string::npos)
      input_layer_ = n.name();

    if (output_target_.empty() && n.name().find("target") != std::string::npos)
      output_target_ = n.name();
      
    if (sample_weights_.empty() && n.name().find("sample_weights") != std::string::npos)
      sample_weights_ = n.name();

    if (learning_phase_.empty() && n.name().find("learning_phase") != std::string::npos)
      learning_phase_ = n.name();

    if (init_node_.empty() && n.name() == "init")
      init_node_ = n.name();

    if (update_node_.empty() && n.name() == "group_deps")
      update_node_ = n.name();

    if (auto_output && n.name() == "group_deps_1")
      output_layer_ = output_layer_ + n.input(0).substr(1) + ", ";
      
    if (n.op() == "Assign" && n.input_size() == 2 && n.input(1).find("Placeholder") != std::string::npos)
    {
      weights_node_.push_back(n.name());
      weights_read_.push_back(n.input(0) + "/read");
      weights_write_.push_back(n.input(1));
    }
  }
  
  graph_ = TF_NewGraph();
  TF_Buffer *tf_graph_def = TF_NewBufferFromString((const void*)ss.str().c_str(), ss.str().size());
  TF_ImportGraphDefOptions* tf_graph_options = TF_NewImportGraphDefOptions();
  TF_Status *tf_status = TF_NewStatus();
  TF_GraphImportGraphDef(graph_, tf_graph_def, tf_graph_options, tf_status);
  
  if (TF_GetCode(tf_status) != TF_OK)
  {
    ERROR(TF_Message(tf_status));
    throw bad_param("representation/tensorflow:file");
  }
  
  TF_SessionOptions *tf_session_options = TF_NewSessionOptions();
  session_ = TF_NewSession(graph_, tf_session_options, tf_status);
  
  if (TF_GetCode(tf_status) != TF_OK)
  {
    ERROR(TF_Message(tf_status));
    throw bad_param("representation/tensorflow:file");
  }
  
  TF_DeleteBuffer(tf_graph_def);
  TF_DeleteImportGraphDefOptions(tf_graph_options);
  TF_DeleteStatus(tf_status);
  TF_DeleteSessionOptions(tf_session_options);
  
  outputs_read_ = cutLongStr(output_layer_);
  
  if (input_layer_.empty())
    throw bad_param("representation/tensorflow:input_layer");

  if (output_layer_.empty())
    throw bad_param("representation/tensorflow:output_layer");

  if (output_target_.empty())
    throw bad_param("representation/tensorflow:target");

  if (update_node_.empty())
    throw bad_param("representation/tensorflow:update_node");
    
  INFO("Model loaded");
  INFO("  Input layer placeholder   : " << input_layer_);
  INFO("  Output layer tensors      : " << outputs_read_);
  INFO("  Target placeholder        : " << output_target_);
  INFO("  Sample weights placeholder: " << sample_weights_);
  INFO("  Learning phase placeholder: " << learning_phase_);
  INFO("  Weight tensors            : " << weights_read_);
  INFO("  Weight placeholders       : " << weights_write_);
  INFO("  Weight nodes              : " << weights_node_);
  INFO("  Init node                 : " << init_node_);
  INFO("  Update node               : " << update_node_);
  
  // Initialize weights
  reset();
}

void TensorFlowRepresentation::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    if (!init_node_.empty())
    {
      INFO("Initializing weights");
      
      // Run session for init node
      TF_Operation *tf_operation = TF_GraphOperationByName(graph_, init_node_.c_str()); 
      TF_Status *tf_status = TF_NewStatus();
      TF_SessionRun(session_, 0,
                    0, 0, 0,           // Inputs
                    0, 0, 0,           // Outputs
                    &tf_operation, 1,  // Operations
                    0, tf_status);
                
      if (TF_GetCode(tf_status) != TF_OK)
      {
        ERROR(TF_Message(tf_status));
        throw bad_param("representation/tensorflow:file");
      }
      
      TF_DeleteStatus(tf_status);
    }
    
    batch_.clear();
    
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
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  
  if (vp)
  {
    // Convert input to tensor
    int64_t shape[] = {1, (int)vp->vector.size()};
    TF_Tensor *input = TF_AllocateTensor(TF_FLOAT, shape, 2, vp->vector.size() * sizeof(float));
    float *data = (float*)TF_TensorData(input);
    for (size_t ii=0; ii < vp->vector.size(); ++ii)
      data[ii] = vp->vector[ii];
      
    // Prepare SessionRun inputs
    std::vector<TF_Output>  input_ops {{TF_GraphOperationByName(graph_, input_layer_.c_str()), 0}};
    std::vector<TF_Tensor*> input_val {input};
      
    if (!learning_phase_.empty())
    {
      // Prepare learning phase tensor
      TF_Tensor *learning = TF_AllocateTensor(TF_BOOL, NULL, 0, sizeof(bool));
      *((bool*)TF_TensorData(learning)) = false;

      input_ops.push_back({TF_GraphOperationByName(graph_, learning_phase_.c_str()), 0});
      input_val.push_back(learning);
    }
    
    // Prepare SessionRun outputs
    std::vector<TF_Output>  output_ops(outputs_read_.size());
    for (size_t oo=0; oo < outputs_read_.size(); ++oo)
      output_ops[oo] = {TF_GraphOperationByName(graph_, outputs_read_[oo].c_str()), 0};
    std::vector<TF_Tensor*> output_val(outputs_read_.size());
      
    // Run session
    TF_Status *tf_status = TF_NewStatus();
    TF_SessionRun(session_, 0, 
                  input_ops.data(), input_val.data(), input_ops.size(),
                  output_ops.data(), output_val.data(), output_ops.size(),
                  0, 0,
                  0, tf_status);
                  
    if (TF_GetCode(tf_status) != TF_OK)
    {
      ERROR(TF_Message(tf_status));
      throw Exception("Could not run prediction graph");
    }
    
    // Allocate result vector
    size_t num_outputs=0;
    for (size_t oo=0; oo < output_val.size(); ++oo)
      num_outputs += TF_Dim(output_val[oo], 1);
    result->resize(num_outputs);
    
    // Convert output tensor to result vector
    size_t residx=0;
    for (size_t oo=0; oo < output_val.size(); ++oo)
    {
      float *data = (float*)TF_TensorData(output_val[oo]);
      for (size_t ii=0; ii < TF_Dim(output_val[oo], 1); ++ii)
        (*result)[residx++] = data[ii];
    }
    
    // Clean up
    for (size_t ii=0; ii < input_val.size(); ++ii)
      TF_DeleteTensor(input_val[ii]);
    for (size_t oo=0; oo < output_val.size(); ++oo)
      TF_DeleteTensor(output_val[oo]);
    TF_DeleteStatus(tf_status);
  }
  else
    throw Exception("representation/tensorflow requires a projector returning a VectorProjection");
    
  if (stddev) *stddev = Vector();
  
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
  // Convert input to tensor
  int64_t input_shape[] = {(int)batch_.size(), (int)batch_[0].first.size()};
  TF_Tensor *input = TF_AllocateTensor(TF_FLOAT, input_shape, 2, batch_.size() * batch_[0].first.size() * sizeof(float));
  float *data = (float*)TF_TensorData(input);
  for (size_t ii=0; ii < batch_.size(); ++ii)
    for (size_t jj=0; jj < batch_[ii].first.size(); ++jj)
      data[ii*batch_[ii].first.size()+jj] = batch_[ii].first[jj];

  // Convert target to tensor
  int64_t target_shape[] = {(int)batch_.size(), (int)batch_[0].second.size()};
  TF_Tensor *target = TF_AllocateTensor(TF_FLOAT, target_shape, 2, batch_.size() * batch_[0].second.size() * sizeof(float));
  data = (float*)TF_TensorData(target);
  for (size_t ii=0; ii < batch_.size(); ++ii)
    for (size_t jj=0; jj < batch_[ii].second.size(); ++jj)
      data[ii*batch_[ii].second.size()+jj] = batch_[ii].second[jj];
      
  // Prepare SessionRun inputs
  std::vector<TF_Output> input_ops  {{TF_GraphOperationByName(graph_, input_layer_.c_str()), 0},
                                     {TF_GraphOperationByName(graph_, output_target_.c_str()), 0}};
  std::vector<TF_Tensor*> input_val {input, target};
                                      
  if (!learning_phase_.empty())
  {
    // Prepare learning phase tensor
    TF_Tensor *learning = TF_AllocateTensor(TF_BOOL, NULL, 0, sizeof(bool));
    *((bool*)TF_TensorData(learning)) = true;

    input_ops.push_back({TF_GraphOperationByName(graph_, learning_phase_.c_str()), 0});
    input_val.push_back(learning);
  }
  if (!sample_weights_.empty())
  {
    // Prepare sample weight tensor
    TF_Tensor *weights = TF_AllocateTensor(TF_FLOAT, input_shape, 1, batch_.size() * sizeof(float));
    float *data = (float*)TF_TensorData(weights);
    for (size_t ii=0; ii < batch_.size(); ++ii)
      data[ii] = 1.;

    input_ops.push_back({TF_GraphOperationByName(graph_, sample_weights_.c_str()), 0});
    input_val.push_back(weights);
  }
                                      
  // Run session for update node
  TF_Operation *tf_operation = TF_GraphOperationByName(graph_, update_node_.c_str()); 
  TF_Status *tf_status = TF_NewStatus();
  TF_SessionRun(session_, 0, 
                input_ops.data(), input_val.data(), input_ops.size(),
                0, 0, 0,
                &tf_operation, 1,
                0, tf_status);
                
  if (TF_GetCode(tf_status) != TF_OK)
  {
    ERROR(TF_Message(tf_status));
    throw Exception("Could not run learning graph");
  }

  // Clean up
  for (size_t ii=0; ii < input_val.size(); ++ii)
    TF_DeleteTensor(input_val[ii]);
  TF_DeleteStatus(tf_status);

  batch_.clear();
}

void TensorFlowRepresentation::batchRead(size_t sz)
{
  // Allocate input tensor
  int64_t shape[] = {(int)sz, (int)inputs_};
  input_ = TF_AllocateTensor(TF_FLOAT, shape, 2, sz * inputs_ * sizeof(float));
  
  counter_ = 0;
}

void TensorFlowRepresentation::batchWrite(size_t sz)
{
  // Allocate input tensor
  int64_t input_shape[] = {(int)sz, (int)inputs_};
  input_ = TF_AllocateTensor(TF_FLOAT, input_shape, 2, sz * inputs_ * sizeof(float));
  
  // Allocate target tensor
  int64_t target_shape[] = {(int)sz, (int)targets_};
  target_ = TF_AllocateTensor(TF_FLOAT, target_shape, 2, sz * targets_ * sizeof(float));
  
  counter_ = 0;
}

void TensorFlowRepresentation::enqueue(const ProjectionPtr &projection)
{
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  
  if (vp)
  {
    // Convert input to tensor
    float *data = (float*)TF_TensorData(input_) + counter_*inputs_;
    for (size_t ii=0; ii < inputs_; ++ii)
      data[ii] = vp->vector[ii];

    counter_++;
  }
  else
    throw Exception("representation/tensorflow requires a projector returning a VectorProjection");
}

void TensorFlowRepresentation::enqueue(const ProjectionPtr &projection, const Vector &target)
{
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  
  if (vp)
  {
    // Convert input to tensor
    float *data = (float*)TF_TensorData(input_) + counter_*inputs_;
    for (size_t ii=0; ii < inputs_; ++ii)
      data[ii] = vp->vector[ii];

    // Convert target to tensor
    data = (float*)TF_TensorData(target_) + counter_*targets_;
    for (size_t ii=0; ii < targets_; ++ii)
      data[ii] = target[ii];

    counter_++;
  }
  else
    throw Exception("representation/tensorflow requires a projector returning a VectorProjection");
}

void TensorFlowRepresentation::read(Matrix *out)
{
  // Prepare SessionRun inputs
  std::vector<TF_Output>  input_ops {{TF_GraphOperationByName(graph_, input_layer_.c_str()), 0}};
  std::vector<TF_Tensor*> input_val {input_};
    
  if (!learning_phase_.empty())
  {
    // Prepare learning phase tensor
    TF_Tensor *learning = TF_AllocateTensor(TF_BOOL, NULL, 0, sizeof(bool));
    *((bool*)TF_TensorData(learning)) = false;

    input_ops.push_back({TF_GraphOperationByName(graph_, learning_phase_.c_str()), 0});
    input_val.push_back(learning);
  }
  
  // Prepare SessionRun outputs
  std::vector<TF_Output>  output_ops(outputs_read_.size());
  for (size_t oo=0; oo < outputs_read_.size(); ++oo)
    output_ops[oo] = {TF_GraphOperationByName(graph_, outputs_read_[oo].c_str()), 0};
  std::vector<TF_Tensor*> output_val(outputs_read_.size());
    
  // Run session
  TF_Status *tf_status = TF_NewStatus();
  TF_SessionRun(session_, 0, 
                input_ops.data(), input_val.data(), input_ops.size(),
                output_ops.data(), output_val.data(), output_ops.size(),
                0, 0,
                0, tf_status);
                
  if (TF_GetCode(tf_status) != TF_OK)
  {
    ERROR(TF_Message(tf_status));
    throw Exception("Could not run prediction graph");
  }
  
  // Allocate result matrix
  size_t num_outputs=0;
  for (size_t oo=0; oo < output_val.size(); ++oo)
    num_outputs += TF_Dim(output_val[oo], 1);
  out->resize(TF_Dim(output_val[0], 0), num_outputs);
  
  // Convert output tensor to result matrix
  size_t residx=0;
  for (size_t oo=0; oo < output_val.size(); ++oo)
  {
    float *data = (float*)TF_TensorData(output_val[oo]);
    for (size_t ii=0; ii < TF_Dim(output_val[oo], 0); ++ii)
      for (size_t jj=0; jj < TF_Dim(output_val[oo], 1); ++jj)
        (*out)(ii, residx+jj) = data[ii*TF_Dim(output_val[oo], 1)+jj];
    residx += TF_Dim(output_val[oo], 1);
  }
  
  // Clean up
  for (size_t ii=0; ii < input_val.size(); ++ii)
    TF_DeleteTensor(input_val[ii]);
  for (size_t oo=0; oo < output_val.size(); ++oo)
    TF_DeleteTensor(output_val[oo]);
  TF_DeleteStatus(tf_status);
  
//  NOTICE("read from:\n" << (input_.tensor<float, 2>()));
//  NOTICE("read result:\n" << *out);
}

void TensorFlowRepresentation::write()
{
  // Prepare SessionRun inputs
  std::vector<TF_Output> input_ops  {{TF_GraphOperationByName(graph_, input_layer_.c_str()), 0},
                                     {TF_GraphOperationByName(graph_, output_target_.c_str()), 0}};
  std::vector<TF_Tensor*> input_val {input_, target_};
                                      
  if (!learning_phase_.empty())
  {
    // Prepare learning phase tensor
    TF_Tensor *learning = TF_AllocateTensor(TF_BOOL, NULL, 0, sizeof(bool));
    *((bool*)TF_TensorData(learning)) = true;

    input_ops.push_back({TF_GraphOperationByName(graph_, learning_phase_.c_str()), 0});
    input_val.push_back(learning);
  }
  if (!sample_weights_.empty())
  {
    // Prepare sample weight tensor
    int64_t shape[] = {(int)counter_};
    TF_Tensor *weights = TF_AllocateTensor(TF_FLOAT, shape, 1, counter_ * sizeof(float));
    float *data = (float*)TF_TensorData(weights);
    for (size_t ii=0; ii < counter_; ++ii)
      data[ii] = 1.;

    input_ops.push_back({TF_GraphOperationByName(graph_, sample_weights_.c_str()), 0});
    input_val.push_back(weights);
  }
                                      
  // Run session for update node
  TF_Operation *tf_operation = TF_GraphOperationByName(graph_, update_node_.c_str()); 
  TF_Status *tf_status = TF_NewStatus();
  TF_SessionRun(session_, 0, 
                input_ops.data(), input_val.data(), input_ops.size(),
                0, 0, 0,
                &tf_operation, 1,
                0, tf_status);
                
  if (TF_GetCode(tf_status) != TF_OK)
  {
    ERROR(TF_Message(tf_status));
    throw Exception("Could not run learning graph");
  }

  // Clean up
  for (size_t ii=0; ii < input_val.size(); ++ii)
    TF_DeleteTensor(input_val[ii]);
  TF_DeleteStatus(tf_status);
  
//  NOTICE("write to:\n" << (input_.tensor<float, 2>()));
//  NOTICE("target:\n" << (target_.tensor<float, 2>()));
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
    output_ops[oo] = {TF_GraphOperationByName(graph_, weights_read_[oo].c_str()), 0};
  std::vector<TF_Tensor*> output_val(weights_read_.size());
    
  // Run session
  TF_Status *tf_status = TF_NewStatus();
  TF_SessionRun(session_, 0, 
                0, 0, 0,
                output_ops.data(), output_val.data(), output_ops.size(),
                0, 0,
                0, tf_status);
                
  if (TF_GetCode(tf_status) != TF_OK)
  {
    ERROR(TF_Message(tf_status));
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
    float *data = (float*)TF_TensorData(output_val[ii]);
    for (size_t jj=0; jj < weights_shape_[ii].size(); ++jj)
      params_[n+jj] = data[jj];
    n += weights_shape_[ii].size();
  }

  // Clean up
  for (size_t oo=0; oo < output_val.size(); ++oo)
    TF_DeleteTensor(output_val[oo]);
  TF_DeleteStatus(tf_status);

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
    TF_Tensor *w = TF_AllocateTensor(TF_FLOAT, weights_shape_[ii].dims(), weights_shape_[ii].num_dims(), weights_shape_[ii].size() * sizeof(float));
    float *data = (float*)TF_TensorData(w);
    
    for (size_t jj=0; jj < weights_shape_[ii].size(); ++jj)
      data[jj] = params_[n+jj];
    n += weights_shape_[ii].size();
    
    input_ops.push_back({TF_GraphOperationByName(graph_, weights_write_[ii].c_str()), 0});
    input_val.push_back(w);
  }
                                      
  // Run session for weight nodes
  std::vector<TF_Operation*> operations(weights_node_.size());
  for (size_t ii=0; ii < operations.size(); ++ii)
    operations[ii] = TF_GraphOperationByName(graph_, weights_node_[ii].c_str());
  TF_Status *tf_status = TF_NewStatus();
  TF_SessionRun(session_, 0, 
                input_ops.data(), input_val.data(), input_ops.size(),
                0, 0, 0,
                operations.data(), operations.size(),
                0, tf_status);
                
  if (TF_GetCode(tf_status) != TF_OK)
  {
    ERROR(TF_Message(tf_status));
    throw Exception("Could not run weight writing graph");
  }

  // Clean up
  for (size_t ii=0; ii < input_val.size(); ++ii)
    TF_DeleteTensor(input_val[ii]);
  TF_DeleteStatus(tf_status);
}
