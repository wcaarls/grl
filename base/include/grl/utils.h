/** \file utils.h
 * \brief Small utility functions and classes.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
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

#ifndef GRL_UTILS_H_
#define GRL_UTILS_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <pthread.h>
#include <iostream>
#include <grl/vector.h>

#ifdef WIN32
#include <ctime>
struct drand48_data { unsigned char dummy; };
#define srand48_r(x, y) srand(x)
#define drand48_r(x, y) do { *(y) = rand()/(RAND_MAX+1.); } while (0)
#define lrand48() (rand()%RAND_MAX)
void __stdcall Sleep(_In_ unsigned int dwMilliseconds);
#define usleep(x) Sleep(((unsigned int)(x))/1000)
inline int round( double r ) {
    return (int)((r > 0.0) ? (r + 0.5) : (r - 0.5));
}
#endif

#define grl_assert(x) do { if (!(x)) { std::cerr << __FILE__ << ":" << __LINE__ << ": Assertion '" << #x << "' failed" << std::endl; abort(); } } while (0)

namespace grl
{

template<class T>
bool safe_delete(T **obj)
{
    if (*obj)
    {
      delete *obj;
      *obj = NULL;
      return true;
    }
    else
      return false;
}
 
template<class T>
bool safe_delete_array(T **obj)
{
    if (*obj)
    {
      delete[] *obj;
      *obj = NULL;  
      return true;  
    }
    else
      return false;
}

template <typename _inttype>
inline _inttype safe_mod(_inttype X, _inttype Y)
{
	_inttype result = X%Y;
	if (result < 0)
		return result+Y;
	else
		return result;
}

/// Random number generator.
/**
 * Maintains an internal state. For multi-threaded use, use RandGen.
 */
class Rand
{
  protected:
    struct drand48_data rand_data_;

  public:
    Rand()
    {
      srand48_r(lrand48(), &rand_data_);
    }
    
    Rand *clone()
    {
      Rand *r = new Rand();
      r->rand_data_ = rand_data_;
      return r;
    }
    
    void init(long int seed)
    {
      srand48_r(seed, &rand_data_);
    }

    double get()
    {
      double r;
      drand48_r(&rand_data_, &r);
      return r;
    }

    double getUniform(double a, double b)
    {
      return a+get()*(b-a);
    }
    
    Vector getVector(size_t sz)
    {
      Vector v(sz);
      for (size_t ii=0; ii < sz; ++ii)
        v[ii] = getUniform(0, 1);
      return v;
    }

    double getNormal(double mu, double sigma)
    {
      double U1 = get(), U2 = get();

      return sqrt(-2*log(U1))*cos(2*M_PI*U2)*sigma+mu;
    }

    size_t getInteger(size_t ma)
    {
      return lrand48()%ma;
    }
};

/// Random number generator generator.
/*
 * For multi-threaded use. Either retrieve a single-threaded instance(),
 * or call directly to make sure you're always accessing your thread-local
 * version.
 */
class RandGen
{
  protected:
    static pthread_once_t once_;
    static pthread_mutex_t mutex_;
    static pthread_key_t key_;

  public:
    static double get() { return instance()->get(); }
    static double getUniform(double a, double b) { return instance()->getUniform(a, b); }
    static Vector getVector(size_t sz) { return instance()->getVector(sz); }
    static double getNormal(double mu, double sigma) { return instance()->getNormal(mu, sigma); }
    static size_t getInteger(size_t ma) { return instance()->getInteger(ma); }

    static Rand *instance()
    {
      grl_assert(pthread_once(&once_, RandGen::init)==0);

      Rand *instance = (Rand*) pthread_getspecific(key_);
      if (!instance)
      {
        pthread_mutex_lock(&mutex_);
        instance = new Rand();
        grl_assert(pthread_setspecific(key_, instance)==0);
        pthread_mutex_unlock(&mutex_);
      }

      return instance;
    }

  protected:
    static void init(void)
    {
      grl_assert(pthread_key_create(&key_, RandGen::fini)==0);
      pthread_mutex_init(&mutex_, NULL);
    }

    static void fini(void *instance)
    {
      delete (Rand*) instance;
    }
};

template <typename T>
inline bool convert(const std::string& str, T *obj)
{
  std::istringstream iss(str);

  iss >> std::ws >> *obj >> std::ws;
  
  return iss.eof();
}

// Sample from distribution
inline size_t sample(Vector dist)
{
  double sum = 0;
  for (size_t ii=0; ii < dist.size(); ++ii)
    sum += dist[ii];
    
  double r = RandGen::get()*sum;
  
  sum = 0;
  for (size_t ii=0; ii < dist.size(); ++ii)
  {
    sum += dist[ii];
    if (r < sum)
      return ii;
  }
  
  return dist.size()-1;
}

}

#endif /* GRL_UTILS_H_ */
