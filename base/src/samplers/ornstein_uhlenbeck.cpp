/** \file ornstein_uhlenbeck.h
 * \brief Ornstein-Uhlenbeck samplers source file.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2016-09-30
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
#include <grl/grl.h>
#include <grl/samplers/ornstein_uhlenbeck.h>

using namespace grl;

REGISTER_CONFIGURABLE(OrnsteinUhlenbeckSampler)

void OrnsteinUhlenbeckSampler::request(ConfigurationRequest *config)
{
  EpsilonGreedySampler::request(config);

  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("theta", "Theta parameter of Ornstein-Uhlenbeck", theta_, CRP::Configuration));
  config->push_back(CRP("sigma", "Sigma parameter of Ornstein-Uhlenbeck", sigma_, CRP::Configuration));
  config->push_back(CRP("center", "Centering parameter of Ornstein-Uhlenbeck", center_, CRP::Configuration));
}

void OrnsteinUhlenbeckSampler::configure(Configuration &config)
{
  EpsilonGreedySampler::configure(config);

  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  discretizer_->options(&variants_);

  theta_ = config["theta"].v();
  sigma_ = config["sigma"].v();
  center_ = config["center"].v();

  if (theta_.size() != sigma_.size() || sigma_.size() != center_.size() || center_.size() == 0)
    throw bad_param("sampler/ornstein_ohlenbeck:{theta,sigma,center}");

  discretizer_->convert(center_, mai_);
}

void OrnsteinUhlenbeckSampler::reconfigure(const Configuration &config)
{
  EpsilonGreedySampler::reconfigure(config);
}

OrnsteinUhlenbeckSampler *OrnsteinUhlenbeckSampler::clone()
{
  OrnsteinUhlenbeckSampler *egs = new OrnsteinUhlenbeckSampler(*this);
  egs->rand_ = rand_->clone();
  return egs;
}

size_t OrnsteinUhlenbeckSampler::sample(const Vector &values, TransitionType &tt) const
{
  if (rand_->get() < epsilon_)
  {
    tt = ttExploratory;

    // convert array index to values
    Vector smp = variants_[mai_];
    TRACE(smp);

    // pertub action according to Ornstein-Uhlenbeck
    for (int i = 0; i < smp.size(); i++)
      smp[i] = smp[i] + theta_[i] * (center_[i] - smp[i])+ sigma_[i] * rand_->getNormal(0, 1);
    TRACE(smp);

    // find nearest discretized sample (min-max bounds preserved automatically)
    discretizer_->discretize(smp);
    TRACE(smp);

    // find value index of the sample and keep it for nex run
    discretizer_->convert(smp, mai_);
  }
  else
    mai_ = GreedySampler::sample(values, tt);

  return mai_;
}

