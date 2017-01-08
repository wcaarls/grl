/** \file multi.cpp
 * \brief Multi experiment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-12-08
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

#include <unistd.h>
#include <iostream>
#include <iomanip>

#include <grl/experiments/multi.h>

using namespace grl;

REGISTER_CONFIGURABLE(MultiExperiment)

void MultiExperiment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("instances", "Number of experiments to run in parallel", instances_, CRP::Configuration, 1));
  config->push_back(CRP("experiment", "experiment", "Experiment to run", prototype_));
}

void MultiExperiment::configure(Configuration &config)
{
  instances_ = config["instances"];
  prototype_ = (Experiment*)config["experiment"].ptr();
  
  for (size_t ii=0; ii < instances_; ++ii)
  {
    Experiment *exp = (Experiment*)prototype_->clone();
    Configuration config;
    std::ostringstream oss;
    oss << "@" << ii;
    config.set("identity", oss.str());
    exp->reconfigure(config);
  
    experiments_.push_back(new MultiExperimentInstance(exp));
  }
}

void MultiExperiment::reconfigure(const Configuration &config)
{
}

void MultiExperiment::run()
{
  for (size_t ii=0; ii < instances_; ++ii)
    experiments_[ii]->start();
    
  for (size_t ii=0; ii < instances_; ++ii)
    experiments_[ii]->join();
}
