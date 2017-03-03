/** \file random.cpp
 * \brief Configurable class of RandGen header file.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@ctudelft.nl>
 * \date      2016-06-27
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Ivan Koryakovskiy
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

#ifndef GRL_RANDOM_GENERATOR_H_
#define GRL_RANDOM_GENERATOR_H_

#include <grl/utils.h>
#include <grl/grl.h>
#include <grl/configurable.h>

namespace grl
{

class RandomGenerator : public Configurable
{
  public:
    virtual ~RandomGenerator() { }

    virtual double get(double *param = NULL) const = 0;

  protected:
    RandGen *rand_;
};

class RandomGeneratorUniform : public RandomGenerator
{
  public:
    TYPEINFO("random_generator/uniform", "Uniform Random generator")

  public:
    RandomGeneratorUniform() : lower_(0.), upper_(1.) { rand_ = new RandGen(); }
    ~RandomGeneratorUniform() { delete rand_; }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From RandomGenerator
    virtual double get(double *param = NULL) const;

  protected:
    double lower_, upper_;
};

class RandomGeneratorUniformInteger : public RandomGenerator
{
  public:
    TYPEINFO("random_generator/uniform_integer", "Uniform Integer Random generator")

  public:
    RandomGeneratorUniformInteger() : ma_(10) { rand_ = new RandGen(); }
    ~RandomGeneratorUniformInteger() { delete rand_; }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From RandomGenerator
    virtual double get(double *param = NULL) const;

  protected:
    int ma_;
};

class RandomGeneratorNormal : public RandomGenerator
{
  public:
    TYPEINFO("random_generator/normal", "Normal Random generator")

  public:
    RandomGeneratorNormal() : mu_(0.), sigma_(1.) { rand_ = new RandGen(); }
    ~RandomGeneratorNormal() { delete rand_; }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From RandomGenerator
    virtual double get(double *param = NULL) const;

  protected:
    double mu_, sigma_;
};

class RandomGeneratorOrnsteinUhlenbeck : public RandomGenerator
{
  public:
    TYPEINFO("random_generator/ornstein_uhlenbeck", "Ornstein-Uhlenbeck Random generator")

  public:
    RandomGeneratorOrnsteinUhlenbeck() : center_(0.), theta_(0.05), sigma_(0.15) { rand_ = new RandGen(); }
    ~RandomGeneratorOrnsteinUhlenbeck() { delete rand_; }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From RandomGenerator
    virtual double get(double *param = NULL) const;

  protected:
    double center_, theta_, sigma_;
};

}

#endif /* GRL_RANDOM_GENERATOR_H_ */
