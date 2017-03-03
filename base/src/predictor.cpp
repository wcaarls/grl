/** \file td.cpp
 * \brief Generic predictor source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-09-25
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

#include <cmath>
#include <grl/predictor.h>

using namespace grl;

void Predictor::request(ConfigurationRequest *config)
{
  config->push_back(CRP("importer", "importer.static", "Optional importer for pre-training", importer_, true));
  config->push_back(CRP("exporter", "exporter", "Optional exporter for transition log (supports observation, action, reward, next_observation, next_action)", exporter_, true));
}

void Predictor::configure(Configuration &config)
{
  importer_ = (Importer*) config["importer"].ptr();
  exporter_ = (Exporter*) config["exporter"].ptr();
  if (exporter_)
  {
    exporter_->init({"observation", "action", "reward", "next_observation", "next_action"});
    exporter_->open("", false);
    exporter_->open();
  }
}

void Predictor::reconfigure(const Configuration &config)
{
}

void Predictor::update(const Transition &transition)
{
  if (importer_)
  {
    Importer *importer = importer_;
    importer_ = NULL;
  
    NOTICE("Pre-training");
    
    importer->init({"observation", "action", "reward", "next_observation", "next_action"});
    importer->open();
    
    Transition t;
    Vector r;
    while (importer->read({&t.prev_obs.v, &t.prev_action.v, &r, &t.obs.v, &t.action.v}))
    {
      if (!std::isfinite(t.obs[0]))
      {
        t.obs = Observation();
        t.action = Action();
      }
      
      t.reward = r[0];
      CRAWL(t);
      update(t);
    }
    
    finalize();
    
    NOTICE("Pre-training done");
  }

  if (exporter_)
  {
    Transition t = transition;
    if (!t.obs.size())
    {
      t.obs = ConstantVector(t.prev_obs.size(), nan(""));
      t.action = ConstantVector(t.prev_action.size(), nan(""));
    }
  
    exporter_->write({t.prev_obs, t.prev_action, VectorConstructor(t.reward), t.obs, t.action});
  }
}

void Predictor::finalize()
{
}
