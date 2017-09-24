/** \file itc.h
 * \brief Inter-thread communication header.
 *
 * Defines classes that help with inter-thread communication, such as
 * itc::Thread base class and itc::SharedVariable shared-variable class.
 *
 * \author    Wouter Caarls <w.caarls@tudelft.nl>
 * \date      2013-06-24
 *
 * \copyright \verbatim
 * Copyright (c) 2013, Wouter Caarls, Delft University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: 
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *       and/or other materials provided with the distribution. 
 *          
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * \endverbatim
 *
 * \mainpage
 *
 * ITC (itc.h) is a small inter-thread communication library.
 * It provides three main components:
 * -  \subpage threading, via the itc::Thread class
 * -  \subpage variables, via the itc::SharedVariable class.
 * -  \subpage queues, via the itc::Queue class.
 *
 * \author    Wouter Caarls <w.caarls@tudelft.nl>
 *
 * \page threading Threading 
 * Classes that should be run in their own threads should derive from itc::Thread
 * and override the itc::Thread::run() function. A basic implementation is given
 * below:
 *
 * \code
 * class MyThread : public itc::Thread
 * {
 *   protected:
 *     virtual void run()
 *     {
 *       while (ok())
 *       {
 *         // Do stuff.
 *       }
 *     }
 * };
 *
 * void setup()
 * {
 *   MyThread thread;
 *   thread.start();
 * 
 *   // ...
 * }
 *
 * \endcode
 *
 * \page variables Shared variables
 *
 * For inter-thread communication, simply create a itc::SharedVariable instantiated
 * to the correct type (preferably in the class that produces it), and provide
 * itc::SharedVariableReference objects to all consumers by calling
 * itc::SharedVariable::reference() on it. Note that no strict produce-consumer
 * relation is required. An example:
 *
 * \code
 * class ProducerThread : public itc::Thread
 * {
 *   private:
 *     itc::SharedVariable<int> var_;
 * 
 *   public:
 *     itc::SharedVariableReference<int> getOutput()
 *     {
 *       return var_.reference();
 *     }
 *
 *     // ...
 * };
 *
 * class ConsumerThread : public itc::Thread
 * {
 *   private:
 *     itc::SharedVariableReference<int> var_;
 * 
 *   public:
 *     void setInput(const itc::SharedVariableReference<int> &var)
 *     {
 *       var_ = var;
 *     }
 *
 *     // ...
 * };
 * 
 * void setup()
 * {
 *   // ...
 *   
 *   consumer.setInput(producer.getOutput());
 * }
 * \endcode
 *
 * There are four ways of interacting with an itc::SharedVariable or itc::SharedVariableReference:
 *  - Through explicit locking
 *    \code
 *    var.readLock();
 *    std::cout << *var << std::endl;
 *    var.unlock();
 *    \endcode
 *  - Through guarded locking
 *    \code
 *    {
 *      itc::WriteGuard guard(var);
 *      (*var)++;
 *    }
 *    \endcode
 *  - Through automatic locking with \ref itc::SharedVariableInterface::get() "get()" and
 *    \ref itc::SharedVariableInterface::set() "set()". Note that this involves copying.
 *    \code
 *    var.set(123);
 *    \endcode
 *  - Through signalled locking with \ref itc::SharedVariableInterface::read() "read()" ,
 *    \ref itc::SharedVariableInterface::write() "write()" and \ref itc::SharedVariableInterface::test() "test()".
 *    This makes sure the same value is not read twice.
 *    However, it does not ensure that all data is read!
 *    \code
 *    std::cout << var.read() << std::endl;
 *    \endcode
 *    The \ref itc::SharedVariableInterface::test() "test()" function is used to test whether a variable has changed and can be used
 *    as a means of signaling between threads.
 *    \code
 *    if(var.test())
 *    {
 *        var.read();
 *        // ...
 *    }
 *    \endcode
 */

#ifndef ITC_H_
#define ITC_H_

#include <stdint.h>
#include <pthread.h>

namespace itc
{

/**
 * \brief Simple threading class.
 *
 * Derive from this and implement the run() function.
 * Start with start() and stop with stop(). run() should terminate when
 * ok() becomes false.
 */
class Thread
{
  private:
    pthread_t thread_id_; ///< Thread identifier.
    bool continue_;       ///< False when thread should stop.

  public:
    /**
     * \brief Constructor
     *
     * \note Does not start the thread. You may do so by calling start() (in the
     *       derived class constructor, if desired).
     */
    Thread() : continue_(false)
    {
    }

    /** \brief Destructor
     *
     * \note Stops the thread if it was still running.
     */
    virtual ~Thread()
    {
      if (continue_)
        stopAndJoin();
    }

    /// Starts the thread.
    void start()
    {
      if (continue_)
        return;

      continue_ = true;
      pthread_create(&thread_id_, NULL, runDelegate, this);
    }

    /// Asks a thread to stop.
    void stop()
    {
      if (!continue_)
        return;

      continue_ = false;
    }
    
    /// Waits for the thread to exit by itself.
    void join()
    {
      pthread_join(thread_id_, NULL);
    }
    
    /// Stops the thread and waits for it to exit.
    void stopAndJoin()
    {
      if (!continue_)
        return;

      continue_ = false;
      pthread_join(thread_id_, NULL);
    }

  protected:
    /// Returns false() when the thread is stopped.
    bool ok()
    {
      return continue_;
    }    

    /**
     * \brief Entry point.
     *
     * Override in derived class.
     * \note Must exit when ok() becomes false.
     */
    virtual void run() = 0;

  private:
    /**
     * \brief Thread start helper function.
     *
     * \param obj Pointer to thread object.
     */
    static void* runDelegate(void *obj)
    {
      Thread *_this = (Thread*) obj;
      _this->run();
      return NULL;
    }
};

/// Read-write lock.
class ReadWriteLockInterface
{
  public:
    virtual ~ReadWriteLockInterface() { }
    
    /// Obtain a read lock.
    virtual void readLock() = 0;
    
    /// Obtain a write lock.
    virtual void writeLock() = 0;
    
    /// Unlock.
    virtual void unlock() = 0;
};

/// Read guard for a ReadWriteLock.
class ReadGuard
{
  private:
    ReadWriteLockInterface &ref_; ///< Lock to guard.

  public:
    /**
     * \brief Constructor.
     *
     * Obtains a read lock from ref.
     * \param ref Lock to guard.
     */
    ReadGuard(ReadWriteLockInterface &ref) : ref_(ref)
    {
      ref_.readLock();
    }

    /**
     * \brief Destructor.
     *
     * Unlocks the ReadWriteLock.
     */
    ~ReadGuard()
    {
      ref_.unlock();
    }
};

/// Write guard for a ReadWriteLock.
class WriteGuard
{
  private:
    ReadWriteLockInterface &ref_; ///< Lock to guard.

  public:
    /**
     * \brief Constructor.
     *
     * Obtains a write lock from ref.
     * \param ref Lock to guard.
     */
    WriteGuard(ReadWriteLockInterface &ref) : ref_(ref)
    {
      ref_.writeLock();
    }

    /**
     * \brief Destructor.
     *
     * Unlocks the ReadWriteLock.
     */
    ~WriteGuard()
    {
      ref_.unlock();
    }
};

/**
 * \brief Base class for shared variables and their references.
 *
 * There are three ways of operation:
 *  - Lock and work through the reference obtained by calling operator*().
 *  - get() / set(), which do the locking automatically.
 *  - read() / write(), which make sure the same value is not read twice.
 *
 * \note Do not call get(), set(), read() or write() when the variable
 *       is already locked. Do not use operator*() when it is unlocked.
 */
template<class T>
class SharedVariableInterface : public ReadWriteLockInterface
{
  public:
    /// Wait for new data.
    virtual void wait() = 0;
    
    /// Mark latest data as read.
    virtual void mark() = 0;
    
    /// Test for existence of new data.
    virtual bool test() = 0;
    
    /// Signal arrival of new data.
    virtual void signal() = 0;
    
    /// Get reference to data.
    virtual T &operator*() = 0;

    /// Member access.
    virtual T *operator->()
    {
      return &**this;
    }

    /**
     * \brief Atomically get the data.
     *
     * \note Do not use under lock.
     */   
    virtual T get()
    {
      ReadGuard guard(*this);
      return **this;
    }
    
    /**
     * \brief Atomically set the data.
     *
     * \param obj New data.
     *
     * \note Do not use under lock.
     */   
    virtual void set(const T &obj)
    {
      WriteGuard guard(*this);
      **this = obj;
    }
    
    /**
     * \brief Atomically get new data.
     *
     * Blocks until new data is available through write().
     * \note Do not use under lock.
     */   
    virtual T read()
    {
      wait();
      
      // Ideally we would actually get the data we were waiting for,
      // but because there's no guarantee about reading all data
      // this works just as well.
      ReadGuard guard(*this);
      mark();
      return **this;
    }
    
    /**
     * \brief Atomically set new data.
     *
     * Signals threads blocking on read().
     * \param obj New data.
     *
     * \note Do not use under lock.
     *       Does not ensure that all data is read!
     */   
    virtual void write(const T &obj)
    {
      WriteGuard guard(*this);
      **this = obj;
      signal();
    }
};

/**
 * \brief Reference to a shared variable.
 *
 * Basically a convenience if you don't want to pass SharedVariable
 * pointers around. Dereference once to get the data.
 */
template<class T>
class SharedVariableReference : public SharedVariableInterface<T>
{
  private:
    SharedVariableInterface<T> *var_; ///< SharedVariable reference.

  public:
    /**
     * \brief Constructor.
     *
     * \param var SharedVariable to reference.
     */
    SharedVariableReference(SharedVariableInterface<T> *var=NULL) : var_(var) { }
  
    void writeLock()
    {
      if (!var_) return;
      var_->writeLock();
    }

    void readLock()
    {
      if (!var_) return;
      var_->readLock();
    }

    void unlock()
    {
      if (!var_) return;
      var_->unlock();
    }

    void wait()
    {
      if (!var_) return;
      var_->wait();
    }

    void mark()
    {
      if (!var_) return;
      var_->mark();
    }

    bool test()
    {
      if (!var_) return false;
      return var_->test();
    }

    void signal()
    {
      if (!var_) return;
      var_->signal();
    }

    T &operator*()
    {
      if (!var_) throw("Dereferencing uninitialized shared variable reference");
      return **var_;
    }
};

/**
 * \brief Shared variable class.
 *
 * Allows communication between threads without race conditions.
 */
template<class T>
class SharedVariable : public SharedVariableInterface<T>
{
  private:
    T *data_;                   ///< Shared data.
    bool owner_;                ///< Whether to destroy data on destruction.
    uintptr_t sequence_;        ///< Sequence number for signallling.

    pthread_rwlock_t rwlock_;   ///< Read-write lock.
    pthread_mutex_t mutex_;     ///< Mutex for signalling.
    pthread_cond_t condition_;  ///< Condition for signalling.
    pthread_key_t key_;         ///< Contains thread-specific sequence numbers for signalling.
        
  public:
    /** 
     * \brief Constructor.
     *
     * \param data Shared data. If NULL, a new object is constructed using the default constructor.
     * \param owner owner If true, the data will be destroyed on destruction of the shared variable.
     */
    SharedVariable(T *data=NULL, bool owner=false) : data_(data), owner_(owner), sequence_(0)
    {
      pthread_rwlock_init(&rwlock_, NULL);
      pthread_mutex_init(&mutex_, NULL);
      pthread_cond_init(&condition_, NULL);
      pthread_key_create(&key_, NULL);
      
      if (!data_)
      {
        data_ = new T();
        owner_ = true; // #ivan Otherwise it is not deleted in destructor
      }
    }

    /**
     * \brief Destructor.
     *
     * If we're the owner, the data will be destroyed here.
     */
    ~SharedVariable()
    {
      if (owner_)
        delete data_;

      pthread_rwlock_destroy(&rwlock_);
      pthread_mutex_destroy(&mutex_);
      pthread_cond_destroy(&condition_);
      pthread_key_delete(key_);
    }
    
    void writeLock()
    {
      pthread_rwlock_wrlock(&rwlock_);
    }

    void readLock()
    {
      pthread_rwlock_rdlock(&rwlock_);
    }

    void unlock()
    {
      pthread_rwlock_unlock(&rwlock_);
    }

    void wait()
    {
      pthread_mutex_lock(&mutex_);
      size_t sequence = (uintptr_t)pthread_getspecific(key_);
      while (sequence_ == sequence) pthread_cond_wait(&condition_, &mutex_);
      pthread_setspecific(key_, (void*)sequence_);
      pthread_mutex_unlock(&mutex_);
    }
    
    void mark()
    {
      pthread_setspecific(key_, (void*)sequence_);
    }

    bool test()
    {
      size_t sequence = (uintptr_t)pthread_getspecific(key_);
      return (sequence_ != sequence);
    }

    void signal()
    {
      pthread_mutex_lock(&mutex_);
      sequence_++;
      pthread_cond_broadcast(&condition_);
      pthread_mutex_unlock(&mutex_);
    }

    T &operator*()
    {
      return *data_;
    }
    
    /// Get a reference to the shared variable to pass along.
    SharedVariableReference<T> reference()
    {
      return SharedVariableReference<T>(this);
    }
    
    SharedVariable(const SharedVariable &&rhs) : data_(rhs.data_), owner_(rhs.owner_), sequence_(rhs.sequence_),
      rwlock_(rhs.rwlock_), mutex_(rhs.mutex_), condition_(rhs.condition_), key_(rhs.key_) { }

  private:
    /// Deny copying.
    SharedVariable(const SharedVariable &rhs) { }
    
    /// Deny assignment.
    SharedVariable &operator=(const SharedVariable &rhs) { return *this; }
    
};

} // namespace itc

#endif // ITC_H_
