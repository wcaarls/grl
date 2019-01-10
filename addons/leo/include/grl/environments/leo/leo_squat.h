#ifndef GRL_LEO_SQUAT_ENVIRONMENT_H_
#define GRL_LEO_SQUAT_ENVIRONMENT_H_

#include <leo.h>
#include <grl/environments/odesim/environment.h>
#include <LeoBhWalkSym.h>
#include <STGLeo.h>
#include <STGLeoSim.h>
#include <ThirdOrderButterworth.h>

namespace grl
{

class CLeoBhSquat: public CLeoBhBase
{
  public:
    TYPEINFO("behavior/leo_squat_sym", "Leo squatting behavior with symmetrical switchers of observations")

    CLeoBhSquat() : direction_(-1), prev_direction_(-1), squat_counter_(0), time_of_dir_change_(0),
      min_hip_height_(0), max_hip_height_(0) {}

    virtual void resetState(double time0);
    virtual double calculateReward();
    virtual void parseLeoState(const CLeoState &leoState, Vector &obs);
    virtual void parseLeoAction(const Action &action, Action &target_action);
    virtual bool isDoomedToFall(CLeoState* state, bool report);
    virtual std::string getProgressReport(double trialTime);

    void setDirection(int direction) { direction_ = direction; }
    void updateDirection(double time);
    void getCOM(const double *x, double &hipHeight, double &hipPos) const;

  protected:
    bool isSitting() const;
    bool isStanding() const;
    void getHipHeight(const double *x, double &hipHeight, double &hipPos) const;

  protected:
    int direction_, prev_direction_;
    int squat_counter_;
    double time_of_dir_change_;
    std::vector<double> up_time_, down_time_;
    double prev_hip_height_, prev_hip_pos_, hip_height_, hip_pos_, min_hip_height_, max_hip_height_;
};

/// Squatting Leo robot
class LeoSquatEnvironment: public LeoBaseEnvironment
{
  public:
    TYPEINFO("environment/leo_squat", "Leo squatting environment")

  public:
    LeoSquatEnvironment();
    ~LeoSquatEnvironment() {}

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);

    // From Environment
    virtual void start(int test, Observation *obs);
    virtual double step(const Action &action, Observation *obs, double *reward, int *terminal);
    virtual void report(std::ostream &os) const;

  protected:
    //CLeoBhSquat *bh_;
    double      com_[3];

};

}

#endif /* GRL_LEO_SQUAT_ENVIRONMENT_H_ */
