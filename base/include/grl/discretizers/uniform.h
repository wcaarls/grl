/*
 * discretizer.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef UNIFORM_H_
#define UNIFORM_H_

#include <grl/configurable.h>
#include <grl/discretizer.h>

namespace grl
{

/// Uniform discretization
class UniformDiscretizer : public Discretizer
{
  public:
    TYPEINFO("discretizer/uniform")

  protected:
    std::vector<Vector> values_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Discretizer
    virtual UniformDiscretizer* clone();
    virtual void options(std::vector<Vector> *out) const;
};

}

#endif /* UNIFORM_H_ */
