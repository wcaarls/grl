/*
 * mutex.h
 *
 *  Created on: Jul 30, 2012
 *      Author: wcaarls
 */

#ifndef GRL_MUTEX_H_
#define GRL_MUTEX_H_

namespace grl {


class Lockable
{
  public:
    virtual ~Lockable() { }
    virtual void lock() = 0;
    virtual void unlock() = 0;
};

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
      return pthread_mutex_trylock(&mutex_);
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

class ReadWriteLock
{
  protected:
    pthread_rwlock_t rwlock_;
    
  public:
    ReadWriteLock()
    {
      pthread_rwlockattr_t attr;
      pthread_rwlockattr_init(&attr);
      pthread_rwlockattr_setkind_np(&attr, PTHREAD_RWLOCK_PREFER_WRITER_NONRECURSIVE_NP);
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

}

#endif /* GRL_MUTEX_H_ */
