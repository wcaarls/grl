/** \file ornstein_uhlenbeck.h
 * \brief Ornstein-Uhlenbeck samplers header file.
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

#ifndef GRL_ORNSTEIN_UHLENBECK_SAMPLER_H_
#define GRL_ORNSTEIN_UHLENBECK_SAMPLER_H_

#include <grl/sampler.h>
#include <grl/utils.h>
#include <grl/grl.h>
#include <grl/discretizer.h>
#include <grl/samplers/greedy.h>
#include <grl/signals/signal_v.h>
#include <grl/samplers/pada.h>

namespace grl
{

/// Maximum search with an Ornstein-Uhlenbeck random chance of non-maximums.
class OrnsteinUhlenbeckSampler : public EpsilonGreedySampler
{
  public:
    TYPEINFO("sampler/ornstein_ohlenbeck", "Maximum search with an Ornstein-Uhlenbeck random chance of non-maximums")

  protected:
    Signal *env_event_;
    Discretizer *discretizer_;
    std::vector<Vector> state_variants_;
    Vector theta_, sigma_, center_;
    mutable size_t state_idx_;

  public:
    OrnsteinUhlenbeckSampler() { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Sampler
    virtual OrnsteinUhlenbeckSampler *clone();
    virtual size_t sample(const Vector &values, TransitionType &tt) const;
};

class OrnsteinUhlenbeckSampler2 : public OrnsteinUhlenbeckSampler
{
  public:
    TYPEINFO("sampler/ornstein_ohlenbeck2", "Maximum search with an Ornstein-Uhlenbeck random chance of non-maximums")

  protected:
    mutable Vector noise_;

  public:
    OrnsteinUhlenbeckSampler2() { }

    // From Sampler
    virtual OrnsteinUhlenbeckSampler2 *clone();
    virtual size_t sample(const Vector &values, TransitionType &tt) const;

  protected:
    virtual void env_event_processor() const;
    virtual void evolve_noise() const;
    virtual void mix_signal_noise(const Vector &in, const Vector &noise, IndexVector &out) const;
};

class EpsilonOrnsteinUhlenbeckSampler : public OrnsteinUhlenbeckSampler2
{
  public:
    TYPEINFO("sampler/epsilon_ornstein_ohlenbeck", "Exploitations are done by greedy action selection without constraints, as in e-greedy. Explorations are done with time-correlated noise, as it is in ou.")
  private:
      double epsilon_;

  public:
    EpsilonOrnsteinUhlenbeckSampler() : epsilon_(0.05) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Sampler
    virtual EpsilonOrnsteinUhlenbeckSampler *clone();
    virtual size_t sample(const Vector &values, TransitionType &tt) const;
};

class PadaOrnsteinUhlenbeckSampler : public OrnsteinUhlenbeckSampler2
{
  public:
    TYPEINFO("sampler/pada_ornstein_ohlenbeck", "Exploitations and exploitations are same as ou, but action is selected from a constrained set, as in pada. ")

  public:
    PadaOrnsteinUhlenbeckSampler() { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Sampler
    virtual PadaOrnsteinUhlenbeckSampler *clone();
    virtual size_t sample(const Vector &values, TransitionType &tt) const;

  private:
    PADASampler pada_;
    mutable IndexVector state_idx_v_;
};


}

#endif /* GRL_ORNSTEIN_UHLENBECK_SAMPLER_H_ */