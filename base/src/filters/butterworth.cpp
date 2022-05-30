/** \file butterworth.cpp
 * \brief Butterworth filter source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2022-05-27
 *
 * \copyright \verbatim
 * Copyright (c) 2022, Wouter Caarls
 * Copyright (c) 2008, Erik Schuitema
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

#include <grl/filters/butterworth.h>

using namespace grl;

REGISTER_CONFIGURABLE(ButterworthFilter)

void ButterworthFilter::request(ConfigurationRequest *config)
{
  config->push_back(CRP("order", "Filter order", order_, CRP::Configuration));
  config->push_back(CRP("sampling_frequency", "Sampling frequency", sampling_frequency_, CRP::Configuration, 0., 1000.));
  config->push_back(CRP("cutoff_frequency", "Cutoff frequencies per dimension", cutoff_frequency_, CRP::Configuration));
}

void ButterworthFilter::configure(Configuration &config)
{
  order_ = config["order"];
  sampling_frequency_ = config["sampling_frequency"];
  cutoff_frequency_ = config["cutoff_frequency"].v();
  
  kin_  = std::vector<Vector>(order_+1);
  kout_ = std::vector<Vector>(order_+1);

  /*
   * The transfer function of a continuous-time third order Butterworth filter is as follows:
   *
   *  H(s) = 1 / ( (s/wc)^3 + 2(s/wc)^2 + 2(s/wc) + 1 )
   *
   * where wc (omega_c) is the cutoff frequency in radians (w = 2*pi*f)
   *
   * Digitizing this transfer function to become H(z) using Tustin's method involves the replacement:
   *
   *  s = ( 2 / T ) * ( ( z - 1 ) / ( z + 1) )
   *
   * where T is the sample period.
   * Expanding this (by hand, muahaha) results in the discrete-time transfer function:
   *
   *  H(z) = ( mKin[0]  + mKin[1]  * z^-1 + mKin[2]  * z^-2 + mKin[3]  * z^-3 ) /
   *         ( mKout[0] + mKout[1] * z^-1 + mKout[2] * z^-2 + mKout[3] * z^-3 )
   *
   * with the coefficients as given below and where mKout[0] == 1.
   *
   * Later, I found the following URL, which also shows derived formulas:
   * http://www.planetanalog.com/showArticle.jhtml?articleID=12802683&pgno=3
   *
   * The same was done for the first and second order filters
   *
   * - Erik Schuitema
   */

  // Take the cutoff frequency into account by using s/w_c instead of s.
  // When using Tustin, this becomes s/w_c = (2/(T*w_c))*((z-1)/(z+1)).
  // Therefore, whenever T appears in the formulas, we use T*w_c = T*2*pi*f_c = (1/f_s)*2*pi*f_c.
  Vector T = 2.0*M_PI*cutoff_frequency_/sampling_frequency_;

  switch(order_)
  {
    case 1:
      {
        Vector normalizeOutput  = T + 2.0;
        kout_[1]  = (T - 2.0)/normalizeOutput;

        kin_[0]   = T/normalizeOutput;
        kin_[1]   = T/normalizeOutput;
      }
      break;
    case 2:
      {
        Vector normalizeOutput  = T*T + 2.0*sqrt(2.0)*T + 4.0;
        kout_[1]  = (2.0*T*T                   - 8.0)/normalizeOutput;
        kout_[2]  = (    T*T - 2.0*sqrt(2.0)*T + 4.0)/normalizeOutput;

        kin_[0]   =     T*T/normalizeOutput;
        kin_[1]   = 2.0*T*T/normalizeOutput;
        kin_[2]   =     T*T/normalizeOutput;
      }
      break;
    case 3:
      {
        Vector normalizeOutput = T*T*T + 4.0*T*T + 8.0*T + 8.0;
        kout_[1]  = (3.0*T*T*T + 4.0*T*T - 8.0*T - 24.0)/normalizeOutput;
        kout_[2]  = (3.0*T*T*T - 4.0*T*T - 8.0*T + 24.0)/normalizeOutput;
        kout_[3]  = (    T*T*T - 4.0*T*T + 8.0*T -  8.0)/normalizeOutput;

        kin_[0]   =   T*T*T/normalizeOutput;
        kin_[1]   = 3*T*T*T/normalizeOutput;
        kin_[2]   = 3*T*T*T/normalizeOutput;
        kin_[3]   =   T*T*T/normalizeOutput;

      }
      break;
  }

  clear();
}

void ButterworthFilter::reconfigure(const Configuration &config)
{
}

Vector ButterworthFilter::filter(const Vector &sample)
{
  for (int i = order_; i>0; i--)   // Now let's hope that the compiler unrolls the loop
  {
    if (first_)
    {
      samplebuffer_[i] = sample;
      filterbuffer_[i] = sample;
    }
    else
    {
      samplebuffer_[i] = samplebuffer_[i-1];
      filterbuffer_[i] = filterbuffer_[i-1];
    }
  }
  first_ = false;
  
  samplebuffer_[0] = sample;

  // The actual filtering step
  filterbuffer_[0] = ConstantVector(sample.size(), 0.);
  for (int iIn=0; iIn<=order_; iIn++)
    filterbuffer_[0] += kin_[iIn]*samplebuffer_[iIn];
  for (int iOut=1; iOut<=order_; iOut++)
    filterbuffer_[0] -= kout_[iOut]*filterbuffer_[iOut];
    
  return filterbuffer_[0];
}

void ButterworthFilter::clear()
{
  first_ = true;
  samplebuffer_ = std::vector<Vector>(order_+1);
  filterbuffer_ = std::vector<Vector>(order_+1);
}
