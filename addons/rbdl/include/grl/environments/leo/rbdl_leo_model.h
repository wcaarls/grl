#ifndef RBDL_LEO_MODEL_H
#define RBDL_LEO_MODEL_H

#include <grl/environment.h>

namespace grl
{

class LeoSandboxModel: public Sandbox
{
  public:
    LeoSandboxModel() : dof_count_(0), target_env_(NULL) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);

    // From Model
//    virtual LeoSandboxModel *clone() const;
    virtual void start(const Vector &hint, Vector *state) = 0;
    virtual double step(const Vector &action, Vector *next) = 0;

    virtual double export_meshup_animation(const Vector &action, const Vector &next, bool append = false) const;

  protected:
    Environment *target_env_;
    DynamicalModel dm_;
    int dof_count_;
    Vector action_min_, action_max_;
};

class LeoSquattingSandboxModel : public LeoSandboxModel
{
  public:
    TYPEINFO("sandbox_model/leo_squatting", "State transition model that integrates equations of motion and augments state vector with additional elements")

  public:
    LeoSquattingSandboxModel() { }

    // From Configurable
//    virtual void request(ConfigurationRequest *config);
//    virtual void configure(Configuration &config);

    // From Model
    virtual LeoSquattingSandboxModel *clone() const;
    virtual void start(const Vector &hint, Vector *state);
    virtual double step(const Vector &action, Vector *next);
};

}

#endif // RBDL_LEO_MODEL_H
