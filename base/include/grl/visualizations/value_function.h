#ifndef GRL_VALUE_FUNCTION_VISUALIZATION_H_
#define GRL_VALUE_FUNCTION_VISUALIZATION_H_

#include <string.h>
#include <pthread.h>

#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/policies/q.h>
#include <grl/visualization.h>

namespace grl
{

class ValueFunctionVisualization : public Visualization
{
  public:
    TYPEINFO("visualization/value_function")

  protected:
    Projector *projector_;
    Representation *representation_;
    QPolicy *policy_;
  
    int state_dims_;
    Vector state_min_, state_max_;
    double epsilon_;
    int points_, dimpoints_, texpoints_;
    unsigned int texture_;
    unsigned char *data_;
    Vector dim_order_;
    double value_min_, value_max_;
    bool updated_;
    std::vector<double> state_;
  
  public:
    ValueFunctionVisualization() : state_dims_(0), points_(0), texture_(0), value_min_(0), value_max_(0), updated_(true) { }
    
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(const Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Visualization
    virtual void draw(); 
    virtual void idle();
    virtual void reshape(int width, int height);
};

} /* namespace grl */

#endif /* GRL_VALUE_FUNCTION_VISUALIZATION_H_ */
