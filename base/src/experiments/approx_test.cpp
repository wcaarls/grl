/** \file approx_test.cpp
 * \brief Approximation test source file.
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

#include <grl/experiments/approx_test.h>

using namespace grl;

REGISTER_CONFIGURABLE(ApproxTestExperiment)

void ApproxTestExperiment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("train_samples", "Number of training samples", train_samples_, CRP::Configuration, 1));
  config->push_back(CRP("test_samples", "Number of test samples", test_samples_, CRP::Configuration, 1));
  config->push_back(CRP("file", "Output file (csv format)", file_));

  config->push_back(CRP("input_min", "Lower limit for drawing samples", min_));
  config->push_back(CRP("input_max", "Upper limit for drawing samples", max_));
  
  config->push_back(CRP("projector", "projector", "Projector (should match representation)", projector_));
  config->push_back(CRP("representation", "representation", "Learned representation", representation_));
  config->push_back(CRP("mapping", "mapping", "Function to learn", mapping_));
}

void ApproxTestExperiment::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  mapping_ = (Mapping*)config["mapping"].ptr();
  
  train_samples_ = config["train_samples"];
  test_samples_ = config["test_samples"];
  file_ = config["file"].str();
  
  min_ = config["input_min"].v();
  max_ = config["input_max"].v();
  
  if (min_.size() != max_.size())
    throw bad_param("experiment/approx_test:{min,max}");
}

void ApproxTestExperiment::reconfigure(const Configuration &config)
{
}

void ApproxTestExperiment::run()
{
  for (size_t ii=0; ii < train_samples_; ++ii)
  {
    Vector in = RandGen::getVector(min_.size()), out;
    in = min_ + in*(max_-min_);

    mapping_->read(in, &out);
    
    ProjectionPtr p = projector_->project(in);
    representation_->write(p, out);
  }
  
  representation_->finalize();
  
  std::ostream *os = &std::cout;
  std::ofstream ofs;
  
  if (file_ != "")
  {
    ofs.open(file_.c_str());
    os = &ofs;
  }
  
  for (size_t ii=0; ii < test_samples_; ++ii)
  {
    Vector in = RandGen::getVector(min_.size()), out, app;
    in = min_ + in*(max_-min_);

    mapping_->read(in, &out);
    
    ProjectionPtr p = projector_->project(in);
    representation_->read(p, &app);
    
    double e = sum(out-app);
    
    for (size_t jj=0; jj < in.size(); ++jj)
      (*os) << in[jj] << ", ";
    for (size_t jj=0; jj < out.size(); ++jj)
      (*os) << out[jj] << ", ";
    for (size_t jj=0; jj < app.size(); ++jj)
      (*os) << app[jj] << ", ";
    
    Matrix Jr = representation_->jacobian(p), Jp = projector_->jacobian(in);
    if (Jr.size() && Jp.size())
    {
      Matrix J = Jr*Jp;
      for (size_t jj=0; jj < J.cols(); ++jj)
        (*os) << J(0, jj) << ", ";
    } 
        
    (*os) << e << std::endl;
  }
}
