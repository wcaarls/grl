/** \file swimmer.cpp
 * \brief Swimmer visualization source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-01-06
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Wouter Caarls
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

#include <grl/visualizations/swimmer.h>

using namespace grl;

REGISTER_CONFIGURABLE(SwimmerVisualization) 

void SwimmerVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("state", "signal/vector", "Swimmer state to visualize", state_));
}

void SwimmerVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
    throw Exception("visualization/swimmer requires a configured visualizer to run");

  state_ = (VectorSignal*)config["state"].ptr();

  // Create window  
  create("Swimmer");
}

void SwimmerVisualization::reconfigure(const Configuration &config)
{
}

void SwimmerVisualization::reshape(int width, int height)
{
  initProjection(-1.1, 1.1, -1.1, 1.1);
}

void SwimmerVisualization::idle()
{
  refresh();
}

void SwimmerVisualization::draw()
{
  clear();
  
  Vector state = state_->get();
  
  if (state.size())
  {
    if (state.size() < 9 || (state.size()-5) % 2)
      throw("visualizer/swimmer requires a dynamics/swimmer state");
      
    const int d = (state.size()-5)/2;
    ColumnVector masses = ConstantVector(d, 1.);
    ColumnVector lengths = ConstantVector(d, 1.);

    Matrix Q = -Matrix::Identity(d, d);
    Q.topRightCorner(d-1, d-1) += Matrix::Identity(d-1, d-1);
    Q.bottomRows(1) = masses.transpose();
    Matrix A = Matrix::Identity(d, d);
    A.topRightCorner(d-1, d-1) += Matrix::Identity(d-1, d-1);
    A(d-1, d-1) = 0.;
    Matrix P = Q.inverse()*(A*diagonal(lengths)) / 2.;

    ColumnVector theta = state.middleCols(2, d).transpose();
    ColumnVector cth = theta.array().cos();
    ColumnVector sth = theta.array().sin();

    Matrix M = P - 0.5 * diagonal(lengths);
      
    double x=state[0]+M.row(0)*cth, y=state[1]+M.row(0)*sth;
    
    for (size_t ii=0; ii < d; ++ii)
    {
      double phi = state[2+ii];
    
      drawLink(x/10, y/10, (x+cos(phi))/10, (y+sin(phi))/10);
      if (ii)
        drawJoint(x/10, y/10);
        
      x += cos(phi);
      y += sin(phi);
    }
  }

  swap();
}
