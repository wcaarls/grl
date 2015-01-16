/*
 * utils.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>

#define grl_assert(x) do { if (!(x)) { std::cerr << __FILE__ << ":" << __LINE__ << ": Assertion '" << #x << " failed" << std::endl; abort(); } } while (0)

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
    static double getNormal(double mu, double sigma) { return instance()->getNormal(mu, sigma); }
    static double getInteger(size_t ma) { return instance()->getInteger(ma); }

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

}

#endif /* UTILS_H_ */
