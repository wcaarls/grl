/*
 * llr_representation.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef LLR_REPRESENTATION_H_
#define LLR_REPRESENTATION_H_

#include <eigen3/Eigen/Dense>

#include <grl/projector.h>
#include <grl/representation.h>

namespace grl
{

/// Locally linear regression.
class LLRRepresentation : public Representation
{
  public:
    TYPEINFO("representation/llr")
    
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Matrix;
    typedef Eigen::Matrix<double, 1, Eigen::Dynamic>              RowVector;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1>              ColumnVector;
    
  protected:
    SampleProjector *projector_;
    double ridge_regression_factor_;
    size_t outputs_;

  public:
    LLRRepresentation() : projector_(NULL), ridge_regression_factor_(0.00001), outputs_(1) { }
  
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
