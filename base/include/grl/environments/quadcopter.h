/** \file quadcopter.cpp
 * \brief Quadcopter dynamics header file
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2020-04-14
 *
 * \copyright \verbatim
 * Copyright(c) 2017 Abhijit Majumdar
 * Copyright(c) 2020 Wouter Caarls
 *
 * Based on
 * https://github.com/abhijitmajumdar/Quadcopter_simulator/blob/master/quadcopter.py
 * which carries the following license:
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files(the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions :
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef GRL_QUADCOPTER_ENVIRONMENT_H_
#define GRL_QUADCOPTER_ENVIRONMENT_H_

#include <grl/environment.h>

namespace grl
{

/// Quadcopter propeller
class Propeller
{
  protected:
    double dia_, pitch_;
    char thrust_unit_;

  public:
    Propeller() { }
    Propeller(double dia, double pitch, char thrust_unit='N') : dia_(dia), pitch_(pitch), thrust_unit_(thrust_unit) { }

    double getThrust(double speed) const
    {
      // From http ://www.electricrcaircraftguy.com/2013/09/propeller-static-dynamic-thrust-equation.html
      double thrust = 4.392e-8 * speed * pow(dia_, 3.5) / sqrt(pitch_);
      thrust  = thrust*(4.23e-4 * speed * pitch_);
      if (thrust_unit_ != 'N')
        thrust *= 0.101972;
      return thrust;
    }
};

/// Abhijit Majumdar's Quadcopter dynamics
class QuadcopterDynamics : public Dynamics
{
  public:
    TYPEINFO("dynamics/quadcopter", "Quadcopter dynamics")
    
  protected:
    double g_, b_, L_, r_, weight_, prop_dia_, prop_pitch_, prop_weight_;
    Vector limits_;

    std::vector<Propeller> propellers_;
    Eigen::Matrix3d I_, invI_;

  public:
    QuadcopterDynamics() : g_(9.81), b_(0.0245), L_(0.3), r_(0.1), weight_(1.2), prop_dia_(10.), prop_pitch_(4.5), prop_weight_(1.2)
    {
      limits_ = VectorConstructor(0., 0.);
    }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Dynamics
    virtual void eom(const Vector &state, const Vector &actuation, Vector *xd) const;

  protected:
    Eigen::Matrix3d rotationMatrix(double theta, double phi, double gamma) const;
};

/// Quadcopter hovering task with quadratic costs
class QuadcopterRegulatorTask : public RegulatorTask
{
  public:
    TYPEINFO("task/quadcopter/regulator", "Quadcopter regulator task")
    
  protected:
    Vector action_range_, limits_;
    int wrap_;
    double time_reward_, limit_penalty_;

  public:
    QuadcopterRegulatorTask()
    {
      start_ = VectorConstructor(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
      goal_ = VectorConstructor(0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0);
      stddev_ = VectorConstructor(0.1, 0.1, 0.1, 0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0);
      q_ = VectorConstructor(1, 1, 1, 0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0);
      r_ = VectorConstructor(1e-9, 1e-9, 1e-9, 1e-9);
      timeout_ = 9.99;
      action_range_ = VectorConstructor(3000, 9000);
      limits_ = VectorConstructor(1.,0.);
      wrap_ = 0;
      time_reward_ = 0;
      limit_penalty_ = 1000;
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const;
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual bool invert(const Observation &obs, Vector *state, double time=0.) const;
    
  protected:
    Vector wrap(const Vector &state) const;
    bool failed(const Vector &state) const;
};

}

#endif /* GRL_QUADCOPTER_ENVIRONMENT_H_ */
 