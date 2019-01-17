/** \file pinball.h
 * \brief Pinball environment definitions.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-12
 *
 * \copyright \verbatim
 * Copyright (c) 2015, Wouter Caarls
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

#ifndef GRL_PINBALL_ENVIRONMENT_H_
#define GRL_PINBALL_ENVIRONMENT_H_

#include <grl/environment.h>

namespace grl
{

class Point
{
  protected:
    double x_, y_;

  public:
    Point(double x=0, double y=0) : x_(x), y_(y) { }
    
    double x() const { return x_; }
    double y() const { return y_; }
    double d() const { return sqrt(x_*x_+y_*y_); }
    
    double sum() const { return x_+y_; }
    
    Point operator+ (const Point &rhs) const
    {
      return Point(x() + rhs.x(), y()+rhs.y());
    }
    
    Point &operator +=(const Point &rhs)
    {
      *this = *this + rhs;
      return *this;
    }
    
    Point operator- ()
    {
      return Point(-x(), -y());
    }
    
    Point operator- (const Point &rhs) const
    {
      return Point(x() - rhs.x(), y()-rhs.y());
    }
    
    Point &operator -=(const Point &rhs)
    {
      *this = *this - rhs;
      return *this;
    }
    
    Point operator* (const double &rhs) const
    {
      return Point(x() * rhs, y()*rhs);
    }
    
    Point &operator *=(const double &rhs)
    {
      *this = *this * rhs;
      return *this;
    }

    Point operator/ (const double &rhs) const
    {
      return Point(x() / rhs, y()/rhs);
    }
    
    Point &operator /=(const double &rhs)
    {
      *this = *this / rhs;
      return *this;
    }

    Point operator* (const Point &rhs) const
    {
      return Point(x() * rhs.x(), y()*rhs.y());
    }
    
    Point &operator *=(const Point &rhs)
    {
      *this = *this * rhs;
      return *this;
    }
};

std::ostream& operator<<(std::ostream& os, const Point& obj)
{
  os << "(" << obj.x() << ", " << obj.y() << ")";
  return os;
}

class Ball
{
  protected:
    Point pos_, vel_;
    double radius_;
    
  public:
    Ball(Point pos, Point vel, double radius) : pos_(pos), vel_(vel), radius_(radius) { }
    
    Point pos() const { return pos_; }
    Point vel() const { return vel_; }
    double radius() const { return radius_; }
    
    void roll(Point acc=Point(), double h=1)
    {
      pos_ += vel_*h;
      vel_ += acc*h;
    }
    
    bool bounce(double r=1)
    {
      vel_ = -vel_*r;
      
      return true;
    }
    
    bool bounce(Point n, double r=1)
    {
      // Check that ball is moving away from normal
      if ((n*vel_).sum() < 0)
      {
        double phi = 2*atan2(n.y(), n.x()) - atan2(vel_.y(), vel_.x()) + M_PI;
        vel_ = Point(cos(phi)*vel_.d(), sin(phi)*vel_.d())*r;
        
        return true;
      }
      
      return false;
    }
};

class Obstacle
{
  protected:
    std::vector<Point> points_;

  public:
    // Add in counter-clockwise order
    void addPoint(const Point &x)
    {
      points_.push_back(x);
    }
    
    // Check whether the ball is in collision with the obstacle.
    // If so, bounce ball along the normal of the side that it
    // collided with.
    bool collide(Ball &ball, double r=1) const
    {
      bool collision = false;
    
      for (size_t ii=0; ii < points_.size(); ++ii)
      {
        Point a, b=points_[ii];
      
        if (ii)
          a = points_[ii-1];
        else
          a = points_[points_.size()-1];
          
        Point p = ball.pos();
        
        Point  l = (b-a);                   // Line segment
        double ab = l.d();                  // Length of line segment
        double t = ((p-a)*l).d() / (ab*ab); // t for closest a+t*l
        
        double d;
        
        if (t < 0)
          d = (p-a).d();                    // Off a's side
        else if (t > 1)
          d = (p-b).d();                    // Off b's side
        else
          d = (p - (a + l * t)).d();        // In between
        
        if (d < ball.radius())
          collision |= ball.bounce(Point(l.y(), -l.x()), r);
      }
      
      return collision;
    }
};

class PinballModel : public Model
{
  public:
    enum StateDimIndex { siX, siY, siXd, siYd, siTime };
    enum ActionDimIndex { aiXdd, aiYdd };

    TYPEINFO("model/pinball", "Model of a ball on a plate");
    
  public:
    std::vector<Obstacle> obstacles_;
    double radius_, restitution_, tau_;
    int steps_, maze_;

  public:
    PinballModel() : radius_(0.01), restitution_(0.5), tau_(0.05), steps_(5), maze_(0) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Model
    virtual double step(const Vector &state, const Vector &actuation, Vector *next) const;
};

/// Pinball movement task.
class PinballMovementTask : public Task
{
  public:
    TYPEINFO("task/pinball/movement", "Pinball movement task")

  public:
    double tolerance_;
  
  public:
    PinballMovementTask() : tolerance_(0.05) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void start(int test, Vector *state) const;
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const;
    virtual bool invert(const Observation &obs, Vector *state, double time=0.) const;

  protected:
    bool succeeded(const Vector &state) const;
};

/// Pinball movement task with quadratic costs
class PinballRegulatorTask : public RegulatorTask
{
  public:
    TYPEINFO("task/pinball/regulator", "Pinball regulator task")

  public:
    PinballRegulatorTask()
    {
      start_ = VectorConstructor(0.1, 0.1, 0., 0.);
      goal_ = VectorConstructor(0.9, 0.9, 0., 0.);
      stddev_ = VectorConstructor(0.01, 0.01, 0., 0.);
      q_ = VectorConstructor(5, 1, 0, 0);
      r_ = VectorConstructor(0.01, 0.01);
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual bool invert(const Observation &obs, Vector *state, double time=0.) const;
};

}

#endif /* GRL_PINBALL_ENVIRONMENT_H_ */
