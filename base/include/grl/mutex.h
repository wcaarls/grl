/** \file mutex.h
 * \brief Mutual exclusion locks, conditions, etc.
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
#ifndef GRL_MUTEX_H_
#define GRL_MUTEX_H_

#include <functional>
#include <list>
#include <pthread.h>

#include <grl/compat.h>

namespace grl {

/// Object that may be locked by a Guard.
class Lockable
{
  public:
    virtual ~Lockable() { }
    virtual void lock() = 0;
    virtual void unlock() = 0;
};

/// Mutual exclusion lock.
class Mutex : public Lockable
{
  protected:
    pthread_mutex_t mutex_;
    
  public:
    Mutex()
    {
      pthread_mutex_init(&mutex_, NULL);
    }
    
    virtual ~Mutex()
    {
      pthread_mutex_destroy(&mutex_);
    }
    
    virtual void lock()
    {
      pthread_mutex_lock(&mutex_);
    }
    
    virtual bool trylock()
    {
      return pthread_mutex_trylock(&mutex_)!=0;
    }
    
    virtual void unlock()
    {
      pthread_mutex_unlock(&mutex_);
    }
    
    virtual pthread_mutex_t *mutex()
    {
      return &mutex_;
    }
};

/// Recursive mutual exclusion lock.
class RecursiveMutex: public Mutex
{
  public:
    RecursiveMutex()
    {
      pthread_mutex_destroy(&mutex_);
      
      pthread_mutexattr_t attr;
      pthread_mutexattr_init(&attr);
      pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
      
      pthread_mutex_init(&mutex_, &attr);
    }        
};

/// Condition variable.
class Condition
{
  protected:
    pthread_cond_t condition_;
    
  public:
    Condition()
    {
      pthread_cond_init(&condition_, NULL);
    }
    
    virtual ~Condition()
    {
      pthread_cond_destroy(&condition_);
    }
    
    /// mutex should be locked
    virtual void wait(Mutex &mutex)
    {
      pthread_cond_wait(&condition_, mutex.mutex());
    }
    
    virtual void timed_wait(Mutex &mutex, struct timespec *tp)
    {
      pthread_cond_timedwait(&condition_, mutex.mutex(), tp);
    }
    
    virtual void signal()
    {
      pthread_cond_signal(&condition_);
    }
    
    virtual void broadcast()
    {
      pthread_cond_broadcast(&condition_);
    }
    
    virtual pthread_cond_t *condition()
    {
      return &condition_;
    }
};

/// Read-write lock.
class ReadWriteLock
{
  protected:
    pthread_rwlock_t rwlock_;
    
  public:
    ReadWriteLock()
    {
      pthread_rwlockattr_t attr;
      pthread_rwlockattr_init(&attr);
#ifndef WIN32
      pthread_rwlockattr_setkind_np(&attr, PTHREAD_RWLOCK_PREFER_WRITER_NONRECURSIVE_NP);
#endif
      pthread_rwlock_init(&rwlock_, &attr);
    }
    
    virtual ~ReadWriteLock()
    {
      pthread_rwlock_destroy(&rwlock_);
    }
    
    virtual void readLock()
    {
      pthread_rwlock_rdlock(&rwlock_);
    }
    
    virtual void writeLock()
    {
      pthread_rwlock_wrlock(&rwlock_);
    }
    
    virtual void unlock()
    {
      pthread_rwlock_unlock(&rwlock_);
    }
    
    virtual pthread_rwlock_t *rwlock()
    {
      return &rwlock_;
    }
};

/// Guard for a Lockable object (e.g. Mutex).
class Guard
{
  protected:
    Lockable &mutex_;
    
  public:
    Guard(Lockable &mutex) : mutex_(mutex)
    {
      mutex_.lock();
    }
    
    ~Guard()
    {
      mutex_.unlock();
    }
};

/// Read guard for a read-write lock.
class ReadGuard
{
  protected:
    ReadWriteLock &rwlock_;
    
  public:
    ReadGuard(ReadWriteLock &rwlock) : rwlock_(rwlock)
    {
      rwlock_.readLock();
    }
    
    ~ReadGuard()
    {
      rwlock_.unlock();
    }
};

/// Write guard for a read-write lock.
class WriteGuard
{
  protected:
    ReadWriteLock &rwlock_;
    
  public:
    WriteGuard(ReadWriteLock &rwlock) : rwlock_(rwlock)
    {
      rwlock_.writeLock();
    }
    
    ~WriteGuard()
    {
      rwlock_.unlock();
    }
};

template<class T>
class Instance   
{
  public:
    typedef std::function<T*(void)> Factory;

  protected:
    pthread_mutex_t mutex_;
    pthread_key_t key_;
    std::list<T*> instances_;
    Factory factory_;

  public:
    Instance(Factory factory=&constructorFactory) : factory_(factory)
    {
      grl_assert(pthread_key_create(&key_, NULL)==0);
      grl_assert(pthread_mutex_init(&mutex_, NULL)==0);
    }
       
    ~Instance()
    {
      pthread_key_delete(key_);
      pthread_mutex_destroy(&mutex_);
    
      for (typename std::list<T*>::iterator it=instances_.begin(); it != instances_.end(); ++it)
        delete *it;
        
      instances_.clear();
    }

    T *instance()
    {
      T *instance = (T*) pthread_getspecific(key_);
      if (!instance)
      {
        pthread_mutex_lock(&mutex_);
        instance = factory_();
        grl_assert(pthread_setspecific(key_, instance)==0);
        pthread_mutex_unlock(&mutex_);
        
        instances_.push_back(instance);
      }
       
      return instance;
    }

    T *operator->()
    {
      return instance();
    }
    
    T &operator*()
    {
      return *instance();
    }
    
  protected:
    static T* constructorFactory()
    {
      return new T();
    }
};   

}

#endif /* GRL_MUTEX_H_ */
