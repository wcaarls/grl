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
#include <iterator>
#include <limits>
#include <grl/compat.h>
#include <grl/vector.h>
#include <grl/mutex.h>

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

    double getOrnsteinUhlenbeck(double prev, double center, double theta, double sigma)
    {
      return prev + theta*(center - prev) + sigma * getNormal(0, 1);
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
    static double getOrnsteinUhlenbeck(double prev, double center, double theta, double sigma) { return instance()->getOrnsteinUhlenbeck(prev, center, theta, sigma); }

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

/// Read a string as a certain type.
template <typename T>
inline bool convert(const std::string& str, T *obj)
{
  std::istringstream iss(str);

  iss >> std::ws >> *obj >> std::ws;
  
  return iss.eof();
}

/// Read a string as a vector.
inline bool convert(const std::string& str, Vector *obj)
{
  std::istringstream iss(str);

  std::vector<double> v;
  iss >> std::ws >> v >> std::ws;
  toVector(v, *obj);
  
  return iss.eof();
}

/// Read a string as a vector.
inline bool convert(const std::string& str, LargeVector *obj)
{
  std::istringstream iss(str);

  std::vector<double> v;
  iss >> std::ws >> v >> std::ws;
  toVector(v, *obj);
  
  return iss.eof();
}

/// Sample from distribution
inline size_t sample(const Vector &dist, double sum)
{
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

/// Sample from distribution
inline size_t sample(const Vector &dist)
{
  double sum = 0;
  for (size_t ii=0; ii < dist.size(); ++ii)
    sum += dist[ii];

  return sample(dist, sum);
}

template <class T>
class FIFOSampler
{
  protected:
    size_t sz_, idx_;
    bool full_;
    std::vector<T> items_;
    mutable ReadWriteLock lock_;

  public:
    FIFOSampler(size_t sz=100) : sz_(sz), idx_(0), full_(false)
    {
      items_.resize(sz_);
    }
    
    void clear()
    {
      WriteGuard guard(lock_);
      
      idx_ = 0;
      full_ = false;
    }
  
    T draw() const
    {
      ReadGuard guard(lock_);
      
      if (items_.empty())
        return T();
      else if (full_)
        return items_[RandGen::getInteger(sz_)];
      else
        return items_[RandGen::getInteger(idx_)];
    }
    
    void add(const T &item)
    {
      WriteGuard guard(lock_);
      
      items_[idx_++] = item;
      if (idx_ == sz_)
      {
        full_ = true;
        idx_ = 0;
      }
    }
};

class timer
{
  private:
    struct timespec start_time_;

  public:
    timer()
    {
      restart();
    }

    void restart()
    {
      clock_gettime(CLOCK_MONOTONIC, &start_time_);
    }

    double elapsed() const
    {
      struct timespec now;
      clock_gettime(CLOCK_MONOTONIC, &now);

      return now.tv_sec - start_time_.tv_sec +
             (now.tv_nsec - start_time_.tv_nsec) / 1000000000.0;
    }

    double elapsed_max() const
    {
      return std::numeric_limits<double>::infinity();
    }

    double elapsed_min() const
    {
      struct timespec res;
      clock_getres(CLOCK_MONOTONIC, &res);
      return res.tv_nsec/1000000000.;
    }
};

///
/// \brief squash
/// \param x - input vector to squash
/// \param f - input vector which specify amount of squashing of a corresponding element of x.
/// For each input vector element indexed by i, f[i] < 0 does shrinking, while f[i] > 0 does stretching.
/// Use f[i] == 0 for no squashing for the element x[i].
/// \return squashed values
///
inline Vector squash(const Vector &x, const Vector &f)
{
  Vector y(x.size());

  for (size_t ii=0; ii < x.size(); ++ii)
  {
    if (!f[ii])
      y[ii] = x[ii];
    else
      y[ii] = (((f[ii]>0)+fabs(1./f[ii]))*x[ii]) /
              (((f[ii]<0)+fabs(1./f[ii]))+copysign(x[ii], f[ii]));
  }
  
  return y;
}

///
/// \brief squash
/// \param x - input vector to squash
/// \param f - a scalar, which specifies squashing for all elements of x.
/// f < 0 does shrinking, while f[i] > 0 does stretching of all elements of x.
/// Use f[i] == 0 for no squashing.
/// \return squashed values
///
inline Vector squash(const Vector &x, double f)
{
  Vector y(x.size());

  for (size_t ii=0; ii < x.size(); ++ii)
  {
    if (!f)
      y[ii] = x[ii];
    else
      y[ii] = (((f>0)+fabs(1./f))*x[ii]) /
              (((f<0)+fabs(1./f))+copysign(x[ii], f));
  }
  
  return y;
}

inline bool str_replace(std::string& str, const std::string& from, const std::string& to)
{
  size_t start_pos = str.find(from);
  if(start_pos == std::string::npos)
    return false;
  str.replace(start_pos, from.length(), to);
  return true;
}

///
/// \brief cutLongStr: parcing a comma-separated or space-separated long string into std::string chuncks
/// \param s
/// \return vector of chuncks in order of appearence
///
inline std::vector<std::string> cutLongStr(const std::string &s)
{
  if (s.empty())
    return std::vector<std::string>();
  else
  {
    struct tokens: std::ctype<char>
    {
      tokens(): std::ctype<char>(get_table()) {}
      static std::ctype_base::mask const* get_table()
      {
        typedef std::ctype<char> cctype;
        static const cctype::mask *const_rc = cctype::classic_table();

        static cctype::mask rc[cctype::table_size];
        std::memcpy(rc, const_rc, cctype::table_size * sizeof(cctype::mask));

        rc[','] = std::ctype_base::space;
        rc[' '] = std::ctype_base::space;
        return &rc[0];
      }
    };

    std::stringstream ss(s);
    ss.imbue(std::locale(std::locale(), new tokens()));
    std::istream_iterator<std::string> begin(ss);
    std::istream_iterator<std::string> end;
    std::vector<std::string> vstrings(begin, end);
    return vstrings;
  }
}

}

#endif /* GRL_UTILS_H_ */
