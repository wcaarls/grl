/*
 * linear_representation.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef LINEAR_REPRESENTATION_H_
#define LINEAR_REPRESENTATION_H_

#include <grl/representation.h>

namespace grl
{

/// Sum of feature activations.
class LinearRepresentation : public ParameterizedRepresentation
{
  public:
    TYPEINFO("representation/parameterized/linear")
    
  protected:
    Vector params_;
    size_t outputs_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From ParameterizedRepresentation
    virtual LinearRepresentation *clone() const;
    virtual double read(const ProjectionPtr &projection, Vector *result) const ;
    virtual void write(const ProjectionPtr projection, const Vector &target, double alpha=1);
    virtual void update(const ProjectionPtr projection, const Vector &delta);
    
    virtual size_t size() const
    {
      return params_.size()/outputs_;
    }
    
    virtual const Vector &params() const
    {
      return params_;
    }
    
    virtual Vector &params()
    {
      return params_;
    }
};

}

#endif /* LINEAR_REPRESENTATION_H_ */
