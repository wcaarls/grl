/*
 * linear_representation.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef GRL_MULTISINE_MAPPING_H_
#define GRL_MULTISINE_MAPPING_H_

#include <grl/representation.h>

namespace grl
{

/// Sum of sines.
class MultisineMapping : public Mapping
{
  public:
    TYPEINFO("mapping/multisine")
    
  protected:
    size_t outputs_, sines_, inputs_;
    
    Vector params_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Mapping
    virtual MultisineMapping *clone() const;
    virtual double read(const ProjectionPtr &projection, Vector *result) const ;
    
  protected:
    inline size_t p(size_t oo, size_t ss, size_t ii, size_t pp) const
    {
      return oo*(sines_*(1 + 2*inputs_)) + ss*(1 + 2*inputs_) + ii*2 + pp;
    }
};

}

#endif /* GRL_MULTISINE_MAPPING_H_ */
