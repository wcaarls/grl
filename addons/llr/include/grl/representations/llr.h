/*
 * llr_representation.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef LLR_REPRESENTATION_H_
#define LLR_REPRESENTATION_H_

#include <grl/projector.h>
#include <grl/representation.h>

namespace grl
{

/// Locally linear regression.
class LLRRepresentation : public Representation
{
  public:
    TYPEINFO("representation/llr")
    
  protected:
    NeighborProjector *projector_;
    double ridge_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Representation
    virtual LLRRepresentation *clone() const;
    virtual double read(const ProjectionPtr &projection, Vector *result) const ;
    virtual void write(const ProjectionPtr projection, const Vector &target, double alpha=1);
    virtual void update(const ProjectionPtr projection, const Vector &delta);
};

}

#endif /* LLR_REPRESENTATION_H_ */
