/** \file breakout.cpp
 * \brief Breakout environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2019-01-07
 *
 * \copyright \verbatim
 * Copyright (c) 2019, Wouter Caarls
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
 
#include <grl/environments/breakout.h>

using namespace grl;

REGISTER_CONFIGURABLE(BreakoutSandbox)
REGISTER_CONFIGURABLE(BreakoutTargetingTask)
REGISTER_CONFIGURABLE(BreakoutVisualization)

// BreakoutSandbox

void BreakoutSandbox::request(ConfigurationRequest *config)
{
  config->push_back(CRP("control_step", "double.control_step", "Control step time", tau_, CRP::Configuration, 0.05, DBL_MAX));
  config->push_back(CRP("integration_steps", "Number of integration steps per control step", (int)steps_, CRP::Configuration, 2));
}

void BreakoutSandbox::configure(Configuration &config)
{
  tau_ = config["control_step"];
  steps_ = config["integration_steps"];

  // Note: treating [m] as [cm], to avoid inelastic collision problem
  // https://www.iforce2d.net/b2dtut/gotchas#slownorebound
  // https://github.com/pybox2d/pybox2d/issues/75
  const float envsz = 100.0;

  // Create world
  b2Vec2 gravity(0.0f, -10.0f);
  world_ = new b2World(gravity);
  
  // Create environment
  b2BodyDef staticBody;
  staticBody.type = b2_staticBody;
  staticBody.position.Set(0.0f, 0.0f);
  environment_ = world_->CreateBody(&staticBody);
  
  b2Vec2 vs[4];
  vs[0].Set(-envsz/2, 0.0);
  vs[1].Set(-envsz/2, envsz);
  vs[2].Set(envsz/2, envsz);
  vs[3].Set(envsz/2, 0);
  
  b2ChainShape chain;
  chain.CreateLoop(vs, 4);
  environment_->CreateFixture(&chain, 0.0);

  // Create cart
  b2BodyDef kinematicBody;
  kinematicBody.type = b2_kinematicBody;
  kinematicBody.position.Set(0.0f, 10.f);
  cart_ = world_->CreateBody(&kinematicBody);
  
  // Create paddle
  b2BodyDef dynamicBody;
  dynamicBody.type = b2_dynamicBody;
  dynamicBody.position.Set(0.0f, 10.f);
  paddle_ = world_->CreateBody(&dynamicBody);
  
  b2PolygonShape box;
  box.SetAsBox(5., 1., b2Vec2(0.0, -0.5), 0.0);
  paddle_->CreateFixture(&box, 5.);
  
  // Create piston
  b2PrismaticJointDef jointDef;
  jointDef.Initialize(cart_, paddle_, cart_->GetWorldCenter(), b2Vec2(0.0, 1.0));
  jointDef.lowerTranslation = 0.0f;
  jointDef.upperTranslation = 1.0f;
  jointDef.enableLimit = true;
  jointDef.maxMotorForce = 1000000.f;
  jointDef.motorSpeed = 0.0f;
  jointDef.enableMotor = true;
  piston_ = (b2PrismaticJoint*)world_->CreateJoint(&jointDef);
  
  // Create ball
  dynamicBody.position.Set(-envsz/2+10.f, 10.f);
  ball_ = world_->CreateBody(&dynamicBody);
  
  b2CircleShape circle;
  circle.m_radius = 2.5f;  

  b2FixtureDef fixture;
  fixture.shape = &circle;
  fixture.density = 0.5f;
  fixture.restitution = 0.6f;
  ball_->CreateFixture(&fixture);
}

void BreakoutSandbox::reconfigure(const Configuration &config)
{
}

void BreakoutSandbox::start(const Vector &hint, Vector *state)
{
  if (!state->size())
    *state = VectorConstructor(-0.4, 0.1, 0., 0.1, 0.1, 0.5, 0., 0., 0.);

  if (state->size() != 9)
    throw Exception("sandbox/breakout requires a task/breakout subclass");

  ball_->SetTransform(       b2Vec2(100*(*state)[0], 100*(*state)[1]), 0);
  cart_->SetTransform(       b2Vec2(100*(*state)[2], 10), 0);
  paddle_->SetTransform(     b2Vec2(100*(*state)[2], 100*(*state)[3]), 0);
  ball_->SetLinearVelocity(  b2Vec2(100*(*state)[4], 100*(*state)[5]));
  cart_->SetLinearVelocity(  b2Vec2(100*(*state)[6], 0));
  paddle_->SetLinearVelocity(b2Vec2(100*(*state)[6], 100*(*state)[7]));
  time_ = 0;
}

double BreakoutSandbox::step(const Vector &actuation, Vector *next)
{
  time_ = time_ + tau_;
  cart_->SetLinearVelocity(b2Vec2(actuation[0]*100, 0));
 
  if (actuation[1] > 0.5)
    piston_->SetMotorSpeed(100.);
    
  for (size_t ii=0; ii < steps_; ++ii)
  {
    if (actuation[1] > 0.5 && ii == steps_/2)
      piston_->SetMotorSpeed(-100.);

    world_->Step(tau_/steps_, 8, 2);
  }
    
  b2Vec2 ballpos = ball_->GetPosition();
  b2Vec2 paddlepos = paddle_->GetPosition();
  b2Vec2 ballvel = ball_->GetLinearVelocity();
  b2Vec2 paddlevel = paddle_->GetLinearVelocity();
    
  next->resize(9);
  (*next)[0] = ballpos.x/100;
  (*next)[1] = ballpos.y/100;
  (*next)[2] = paddlepos.x/100;
  (*next)[3] = paddlepos.y/100;
  (*next)[4] = ballvel.x/100;
  (*next)[5] = ballvel.y/100;
  (*next)[6] = paddlevel.x/100;
  (*next)[7] = paddlevel.y/100;
  (*next)[8] = time_;
  
  return tau_;
}

// BreakoutTargetingTask

void BreakoutTargetingTask::request(ConfigurationRequest *config)
{
  Task::request(config);

  config->push_back(CRP("timeout", "Episode timeout", timeout_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("randomization", "Level of start state randomization", randomization_, CRP::Configuration, 0., 1.));
}

void BreakoutTargetingTask::configure(Configuration &config)
{
  timeout_ = config["timeout"];
  randomization_ = config["randomization"];

  config.set("observation_dims", 5);
  config.set("observation_min", VectorConstructor(-0.5, 0., -0.5, -10, -10));
  config.set("observation_max", VectorConstructor( 0.5, 1.,  0.5,  10,  10));
  config.set("action_dims", 2);
  config.set("action_min", VectorConstructor(-1, 0));
  config.set("action_max", VectorConstructor( 1, 1));
  config.set("reward_min", -100);
  config.set("reward_max", 100);
}

void BreakoutTargetingTask::reconfigure(const Configuration &config)
{
}

void BreakoutTargetingTask::start(int test, Vector *state)
{
  *state = VectorConstructor(-0.4, 0.1, 0., 0.1, 0.1+0.1*randomization_*(test==0)*RandGen::get(), 1+0.5*randomization_*(test==0)*RandGen::get(), 0., 0., 0.);
}

void BreakoutTargetingTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  obs->v.resize(5);
  (*obs)[0] = state[0];
  (*obs)[1] = state[1];
  (*obs)[2] = state[2];
  (*obs)[3] = state[4];
  (*obs)[4] = state[5];
  obs->absorbing = false;
  
  if (succeeded(state) || failed(state))
  {
    *terminal = 2;
    obs->absorbing = true;
  }
  else if (state[8] > timeout_)
    *terminal = 1;
  else
    *terminal = 0;
}

void BreakoutTargetingTask::evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const
{
  // Note: requires gamma>=0.999 for succeeded reward to outperform
  // keeping the ball in play
  if (succeeded(next))
    *reward = 1000;
  else if (failed(next))
    *reward = -1000;
  else
    *reward = 1;
}

bool BreakoutTargetingTask::invert(const Observation &obs, Vector *state) const
{
  state->resize(8);
  (*state)[0] = obs[0];
  (*state)[1] = obs[1];
  (*state)[2] = obs[2];
  (*state)[3] = 0.1;
  (*state)[4] = obs[3];
  (*state)[5] = obs[4];
  (*state)[6] = 0;
  (*state)[7] = 0;
  (*state)[8] = 0;

  return true;
}

bool BreakoutTargetingTask::succeeded(const Vector &state) const
{
  if (state[0] > -0.05 && state[0] < 0.05 && state[1] > 0.9)
    return true;
    
  return false;
}

bool BreakoutTargetingTask::failed(const Vector &state) const
{
  if (state[1] < 0.05)
    return true;
    
  return false;
}

// BreakoutVisualization

void BreakoutVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("state", "signal/vector", "Breakout state to visualize", state_));
}

void BreakoutVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
    throw Exception("visualization/pendulum requires a configured visualizer to run");

  state_ = (VectorSignal*)config["state"].ptr();

  // Create window  
  create("Breakout");
}

void BreakoutVisualization::reconfigure(const Configuration &config)
{
}

void BreakoutVisualization::draw()
{
  clear();

  Vector state = state_->get();

  if (state.size())
  {
    drawLink(state[2]-0.025, state[3]-0.005, state[2]+0.025, state[3]-0.005);
    drawMass(state[0], state[1]);
  }

  swap();
}

void BreakoutVisualization::idle()
{
  refresh();
}

void BreakoutVisualization::reshape(int width, int height)
{
  initProjection(-0.5, 0.5, 0, 1);
}
