/** \file queue.h
 * \brief Thread safe queue header.
 *
 * Defines a thread-safe single-writer multiple-readers queue, Queue.
 *
 * \author    Wouter Caarls <w.caarls@tudelft.nl>
 * \date      2013-10-23
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
 * \page queues Queues
 *
 * Queues provide a strict first-in first-out communications channel, where
 * all data that is written by a writer is read by all readers. To create a
 * queue, simple construct an itc::Queue object with the right queue size.
 * To write data to the queue, you need to get hold of the QueueWriter
 * interface, via itc::Queue::getWriter(). And to read data, you need to add a 
 * reader using itc::Queue::addReader():
 * \code
 * itc::Queue<int> queue(10);
 * itc::QueueWriter<int> writer = queue.getWriter();
 * itc::QueueReader<int> reader = queue.addReader();
 * \endcode
 * Writer and reader interfaces may be freely copied, but should only be used
 * by a single thread. The same getOutput()/setInput() structure as in
 * \ref variables may thus be used.
 *
 * There are two ways of interacting with the queue. To avoid copying, use
 * itc::QueueWriter::get(), itc::QueueWriter::advance(), itc::QueueReader::get()
 * and itc::QueueReader::advance():
 * \code
 * int &wrdata = writer.get();       // Get reference to data.
 * wrdata = 1;                       // Manipulate data.
 * writer.advance();                 // Advance write pointer.
 *
 * const int &rddata = reader.get(); // Get reference to data.
 * std::cout << rddata << std::endl; // Use data.
 * reader.advance();                 // Advance read pointer.
 * \endcode
 * 
 * Or, using syntactic sugar:
 * \code
 * int &wrdata = *writer;
 * wrdata = 1;
 * ++writer;
 *
 * const int &rddata = *reader;
 * std::cout << rddata << std::endl;
 * ++reader;
 * \endcode
 *
 * With small data elements copying isn't a problem, and code can be
 * written more simply using itc::QueueWriter::write() and itc::QueueReader::read():
 * \code
 * writer.write(1);                  // Write data.
 * int rddata = reader.read();       // Read data.
 * \endcode
 *
 * Or, using syntactic sugar:
 * \code
 * writer += 1;
 * int rddata = reader++;
 * \endcode
 */
 
#ifndef ITC_QUEUE_H_
#define ITC_QUEUE_H_
 
#include <list>
#include <pthread.h>

namespace itc {

template<class T> class Queue;
template<class T> class QueueReaderImpl;

/****************************************************************************
 * QueueWriter declaration                                                  *
 ****************************************************************************/

template<class T>
class QueueWriterImpl
{
  private:
    Queue<T> *queue_;
    int ptr_;
    bool active_;
    
  private:
    void _wait();

  public:
    QueueWriterImpl(Queue<T> *queue);
    
    void vacate(int ptr);
    bool vacant(int ptr) { return ptr != ptr_; }
    int ptr()            { return ptr_;        }
    bool active()        { return active_;     }

    bool test();
    void wait();
    
    T& get();
    void set(const T& element);
    void advance();
    
    void write(const T& element);
};

/// Queue writer interface class
template <class T>
class QueueWriter
{
  protected:
    QueueWriterImpl<T> *writer_;

  public:
    QueueWriter() { }
    QueueWriter(QueueWriterImpl<T> *writer) : writer_(writer) { }
    
    /**
     * \brief Tests for the availability of an empty spot in the queue.
     *
     * \returns true if an empty spot is available, false otherwise.
     */
    bool test()                  { return writer_->test();  }
    
    /// Waits until an empty spot in the queue is available.
    void wait()                  { writer_->wait();         }
    
    /** \brief Gets a reference to the next empty spot in the queue.
     *
     * \returns Reference to next empty spot in the queue.
     */
    T& get()                     { return writer_->get();   }
    
    /** \brief Sets the value of the next queue element.
     *
     * \param element Value to set next queue element to.
     * \see operator*().
     */
    void set(const T& element)   { writer_->set(element);   }
    
    /** \brief Allows readers to read next queue element.
     *
     * \note Only valid when next element has first been set.
     * \see set(), operator++().
     */
    void advance()               { writer_->advance();      }
    
    /** \brief Writes a new value to the queue.
     *
     * First sets the next queue element, and then allows readers to read it.
     * \param element Value to set next queue element to.
     * \see set(), advance(), operator+=().
     */
    void write(const T& element) { writer_->write(element); }
    
  public:
    /// Syntactic sugar for get().
    T& operator*()               { return get();            }
    
    /// Syntactic sugar for advance().
    void operator++()            { advance();               }
    
    /// Syntactic sugar for write().
    void operator+=(const T &element) { write(element);     }
};

/****************************************************************************
 * QueueReader declaration                                                  *
 ****************************************************************************/

template <class T>
class QueueReaderImpl
{
  private:
    Queue<T> *queue_;
    int ptr_;
    
  private:
    void _wait();

  public:
    QueueReaderImpl(Queue<T> *queue);
    ~QueueReaderImpl();

    void vacate(int ptr);
    bool vacant(int ptr) { return ptr != ptr_; }

    bool test();
    void wait();
    
    const T &get();
    void advance();
    
    T read();
    
    bool engaged()       { return ptr_ != -1;  }
    void disengage();
    void reengage();
};

/// Queue reader interface class
template <class T>
class QueueReader
{
  protected:
    QueueReaderImpl<T> *reader_;

  public:
    QueueReader() { }
    QueueReader(QueueReaderImpl<T> *reader) : reader_(reader) { }
    
    /**
     * \brief Tests for the availability of a new element in the queue.
     * \returns true if a new element is available, false otherwise.
     */
    bool test()          { return reader_->test(); }
    
    /// Waits until a new element in the queue is available.
    void wait()          { reader_->wait();        }
    
    /** \brief Gets a reference to the current element in the queue.
     *
     * \note If there is no current element, waits until a new element is available.
     * \returns Reference to current element in the queue.
     * \see wait().
     */
    const T &get()       { return reader_->get();  }
    
    /** \brief Marks the current element in the queue as read.
     *
     * \note Invalidates reference obtained through get().
     * \note If there is no current element, waits until a new element is available.
     * \see wait(), get(), operator++().
     */
    void advance()       { reader_->advance();     }

    /** \brief Gets a copy of the next element in the queue.
     *
     * First gets a copy of the next element in the queue, and then marks it as read.
     * \returns Copy of next element in the queue
     * \see get(), advance(), operator++(int).
     */
    T read()             { return reader_->read(); }
    
    /** \brief Disengages from the queue, allowing the write to continue unhindered.
     *
     * \see reengage().
     */
    void disengage()     { reader_->disengage();   }
    
    /** \brief Reattaches to the queue, starting at the last written element.
     *
     * \note If currently engaged, fast-forwards to the last written element.
     * \note If no element has been written yet, subsequent accesses will block
     * until one becomes available.
     * \see disengage().
     */
    void reengage()      { reader_->reengage();    }

  public:
    /// Syntactic sugar for get().
    const T& operator*() { return get();           }
    
    /// Syntactic sugar for advance().
    void operator++()    { advance();              }
    
    /// Syntactic sugar for read().
    T operator++(int)    { return read();          }
};

/****************************************************************************
 * Queue                                                                    *
 ****************************************************************************/

/// Thread-safe single-writer multiple-readers queue
template<class T>
class Queue
{
  friend class QueueWriterImpl<T>;
  friend class QueueReaderImpl<T>;

  public:
    typedef std::list<QueueReaderImpl<T>*> ReaderList;

  private:
    T *data_;
    size_t size_;

    QueueWriterImpl<T> *writer_;
    ReaderList readers_;

    pthread_mutex_t mutex_;
    pthread_cond_t read_condition_;
    pthread_cond_t write_condition_;

  private:
    ReaderList &readers()
    {
      return readers_;
    }

    QueueWriterImpl<T> *writer()
    {
      return writer_;
    }

    void signalReaders()
    {
      pthread_cond_broadcast(&write_condition_);
    }

    void signalWriter()
    {
      pthread_cond_signal(&read_condition_);
    }

    void waitReaders()
    {
      pthread_cond_wait(&read_condition_, &mutex_);
    }

    void waitWriter()
    {
      pthread_cond_wait(&write_condition_, &mutex_); 
    }

    T &at(const int idx)
    {
      return data_[idx];
    }

    size_t size()
    {
      return size_;
    }

    void lock()
    {
      pthread_mutex_lock(&mutex_);
    }
    
    void unlock()
    {
      pthread_mutex_unlock(&mutex_);
    }

    /// Deny copying.
    Queue(const Queue &rhs) { }
    
    /// Deny assignment.
    Queue &operator=(const Queue &rhs) { return *this; }

  public:
    /** \brief Constructor
     *
     * \param size Queue size.
     */
    Queue(size_t size=2) : size_(size)
    {
      data_ = new T[size];
      writer_ = new QueueWriterImpl<T>(this);

      pthread_mutex_init(&mutex_, NULL);
      pthread_cond_init(&read_condition_, NULL);
      pthread_cond_init(&write_condition_, NULL);
    }

    /// Destructor
    ~Queue()
    {
      delete[] data_;
      delete writer_;

      // Readers automatically delete themselves from the list when destroyed
      while (!readers_.empty())
        delete *readers_.begin();

      pthread_mutex_destroy(&mutex_);
      pthread_cond_destroy(&read_condition_);
      pthread_cond_destroy(&write_condition_);
    }
  
    /** \brief Gets queue writer interface.
     *
     * The interface may be copied, but should only be used by a single thread.
     * \note There should be only one writer thread. If there are multiple writers, only QueueWriter<T>::write() is guaranteed to work correctly.
     * \returns Queue writer interface.
     */
    QueueWriter<T> getWriter()
    {
      return QueueWriter<T>(writer_);
    }

    /** \brief Gets queue reader interface.
     *
     * As long as this interface exists, the writer cannot continue unless all queue elements are read through it.
     * The interface may be copied, but should only be used by a single thread.
     * \returns Queue reader interface.
     */
    QueueReader<T> addReader()
    {
      lock();
      QueueReaderImpl<T> *reader = new QueueReaderImpl<T>(this);
      readers_.push_back(reader);
      unlock();

      return QueueReader<T>(reader);
    }
};

/****************************************************************************
 * QueueWriter implementation                                               *
 ****************************************************************************/

template <class T>
inline QueueWriterImpl<T>::QueueWriterImpl(Queue<T> *queue) : queue_(queue), ptr_(0)
{
}

template <class T>
inline void QueueWriterImpl<T>::_wait()
{
  int next = (ptr_+1)%queue_->size();
  typename Queue<T>::ReaderList &readers = queue_->readers();
  for (typename Queue<T>::ReaderList::iterator ri=readers.begin(); ri != readers.end(); ++ri)
    (*ri)->vacate(next);
}

template <class T>
inline void QueueWriterImpl<T>::vacate(int ptr)
{
  while (ptr_ == ptr) queue_->waitWriter();
}

template <class T>
inline bool QueueWriterImpl<T>::test()
{
  bool vacant = true;

  queue_->lock();
  int next = (ptr_+1)%queue_->size();
  typename Queue<T>::ReaderList &readers = queue_->readers();
  for (typename Queue<T>::ReaderList::iterator ri=readers.begin(); ri != readers.end(); ++ri)
    vacant &= (*ri)->vacant(next);
  
  queue_->unlock();

  return vacant;
}

template <class T>
inline void QueueWriterImpl<T>::wait()
{
  queue_->lock();
  _wait();
  queue_->unlock();
}

template <class T>
inline T& QueueWriterImpl<T>::get()
{
  queue_->lock();
  _wait();
  T &element = queue_->at(ptr_);
  queue_->unlock();
  return element;
}

template <class T>
inline void QueueWriterImpl<T>::set(const T& element)
{
  queue_->lock();
  _wait();
  queue_->at(ptr_) = element;
  queue_->unlock();
}

template <class T>
inline void QueueWriterImpl<T>::advance()
{
  queue_->lock();
  _wait();
  ptr_ = (ptr_+1)%queue_->size();
  active_ = true;
  queue_->signalReaders();
  queue_->unlock();
}

template <class T>
inline void QueueWriterImpl<T>::write(const T& element)
{
  queue_->lock();
  _wait();
  queue_->at(ptr_) = element;
  ptr_ = (ptr_+1)%queue_->size();
  active_ = true;
  queue_->signalReaders();
  queue_->unlock();
}

/****************************************************************************
 * QueueReader implementation                                               *
 ****************************************************************************/

template <class T>
inline QueueReaderImpl<T>::QueueReaderImpl(Queue<T> *queue) : queue_(queue)
{
  ptr_ = queue_->writer()->ptr();
}

template <class T>
inline QueueReaderImpl<T>::~QueueReaderImpl()
{
  queue_->lock();
  typename Queue<T>::ReaderList &readers = queue_->readers();
  for (typename Queue<T>::ReaderList::iterator ri=readers.begin(); ri != readers.end(); ++ri)
  {
    if (*ri == this)
    {
      readers.erase(ri);
      break;
    }
  }

  queue_->unlock();
}

template <class T>
inline void QueueReaderImpl<T>::_wait()
{
  queue_->writer()->vacate(ptr_);
}

template <class T>
inline void QueueReaderImpl<T>::vacate(int ptr)
{
  while (ptr_ == ptr) queue_->waitReaders();
}

template <class T>
inline bool QueueReaderImpl<T>::test()
{
  if (!engaged())
    return true;

  queue_->lock();
  bool vacant = queue_->writer()->vacant(ptr_);
  queue_->unlock();   
  return vacant;
}

template <class T>
inline void QueueReaderImpl<T>::wait()
{
  if (!engaged())
  {
    reengage();
    return;
  }

  queue_->lock();
  queue_->writer()->vacate(ptr_);
  queue_->unlock();   
}

template <class T>
inline const T& QueueReaderImpl<T>::get()
{
  if (!engaged())
    reengage();

  queue_->lock();
  _wait();
  T &element = queue_->at(ptr_);
  queue_->unlock();
  return element;
}

template <class T>
inline void QueueReaderImpl<T>::advance()
{
  if (!engaged())
    return;
  
  queue_->lock();
  _wait();
  ptr_ = (ptr_+1)%queue_->size();
  queue_->signalWriter();
  queue_->unlock();
}

template <class T>
inline T QueueReaderImpl<T>::read()
{
  if (!engaged())
    reengage();

  queue_->lock();
  _wait();
  T element = queue_->at(ptr_);
  ptr_ = (ptr_+1)%queue_->size();
  queue_->signalWriter();
  queue_->unlock();
  return element;
}

template <class T>
inline void QueueReaderImpl<T>::disengage()
{
  queue_->lock();
  ptr_ = -1;
  queue_->signalWriter();
  queue_->unlock();
}

template <class T>
inline void QueueReaderImpl<T>::reengage()
{
  queue_->lock();
  
  if (!queue_->writer()->active())
    ptr_ = 0;
  else
    ptr_ = (queue_->writer()->ptr()-1+queue_->size())%queue_->size();
  queue_->signalWriter();
  queue_->unlock();
}

} // namespace itc

#endif // ITC_QUEUE_H_
