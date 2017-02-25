#include <zmq_messenger.h>
#include <pthread.h>
#include <unistd.h>

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

class worker_args
{
  public:
    zmq::context_t  *context;
    const char      *addr;
    zmq_mutex       *mtx;
    char            *buffer;
    unsigned int    buffer_size;
    int             *recv_update;
    worker_args (zmq::context_t *_context, const char *_secondaryAddr, zmq_mutex *_mtx, char *_buffer, unsigned int _buffer_size, int *_recv_update) :
      context(_context), addr(_secondaryAddr), mtx(_mtx), buffer(_buffer), buffer_size(_buffer_size), recv_update(_recv_update)
    {}
};

worker_args *g_args = NULL;

void *worker_routine(void *param)
{
  // Prepare ZMQ subscriber
  int confl = 1;
  worker_args *args = (worker_args*) param;
  zmq::socket_t* subscriber = new zmq::socket_t(*(args->context), ZMQ_SUB);
  subscriber->connect(args->addr);
  subscriber->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl)); // Keep only last message
  subscriber->setsockopt(ZMQ_SUBSCRIBE, "", 0);

  while (1)
  {
    zmq::message_t update;
    bool received = subscriber->recv(&update, ZMQ_DONTWAIT);
    if(received)
    {
      if (args->buffer_size < update.size())
      {
        std::cout << "Error: (ZeromqMessenger) Incomming message of silze " << update.size() << " is too large" << std::endl;
        continue;
      }

      args->mtx->lock();
      memcpy(args->buffer, update.data(), update.size());
      *(args->recv_update) = 1;
      args->mtx->unlock();
    }
  }
  zmq_close(subscriber);
  safe_delete(&subscriber);
  return NULL;
}

void ZeromqMessenger::start(int type, const char* primaryAddr, const char* secondaryAddr, const char* syncAddress)
{
  type_ = type;

  // Prepare our context
  context_ = new zmq::context_t(1);

  if (type_ == ZMQ_REQ)
  {
    primary_ = new zmq::socket_t(*context_, ZMQ_REQ);
    primary_->connect(primaryAddr);
  }
  else if (type_ == ZMQ_REP)
  {
    primary_ = new zmq::socket_t(*context_, ZMQ_REP);
    primary_->bind(primaryAddr);
  }
  else if ((type_ == ZMQ_PUB) || (type_ == ZMQ_SUB))
  {
    int confl = 1;

    // Prepare ZMQ publisher
    primary_ = new zmq::socket_t(*context_, ZMQ_PUB);
    primary_->bind(primaryAddr);
    primary_->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl)); // Keep only last message

    // Prepare ZMQ subscriber
    mtx_ = new zmq_mutex();
    buffer_ = new char [buffer_size_];
    recv_update_ = new int(0);
    g_args = new worker_args(context_, secondaryAddr, mtx_, buffer_, buffer_size_, recv_update_);
    int ret = pthread_create(&worker_, NULL, worker_routine, (void*)g_args);
    if (ret)
    {
      std::cout << "Failed creating zmq worker thread with error " << ret << std::endl;
      abort();
    }
  }
  else
  {
    safe_delete(&context_);
    std::cout << "Unsupported type of zeromq messenger: " << type_ << std::endl;
    abort();
  }

  // If syncronization required...
  if (strcmp(syncAddress, "0") != 0)
  {
    if ((type_ == ZMQ_REQ) || (type_ == ZMQ_PUB))
    {
      connected_ = false;
      syncService_ = new zmq::socket_t(*context_, ZMQ_REP);
      syncService_->bind(syncAddress);
    }

    if ((type_ == ZMQ_REP) || (type_ == ZMQ_SUB))
    {
      // synchronize with a client
      syncService_ = new zmq::socket_t(*context_, ZMQ_REQ);
      syncService_->connect(syncAddress);

      // - send a synchronization request
      zmq::message_t message(0);
      syncService_->send(message);

      // - wait for synchronization reply
      zmq::message_t update;
      syncService_->recv(&update);

      // Third, get our updates and report how many we got
      zmq_close(syncService_);

      connected_ = true;
    }
  }
  else
    connected_ = true;
}

void ZeromqMessenger::sync()
{
  if (connected_)
    return;

  if ((type_ == ZMQ_REQ) || (type_ == ZMQ_PUB))
  {
    if (subscribers_ < subscribers_expected_)
    {
      // - wait for synchronization request
      zmq::message_t update;
      if (syncService_->recv(&update, ZMQ_DONTWAIT))
      {
        // - send synchronization reply
        zmq::message_t message(0);
        syncService_->send(message);
        subscribers_++;
      }
    }
    if (subscribers_ == subscribers_expected_)
      connected_ = true;
  }
}

bool ZeromqMessenger::send(const void* data, unsigned int size) const
{
  zmq::message_t message(size);
  memcpy(message.data(), data, size);
  return primary_->send(message);
}

bool ZeromqMessenger::recv(void *data, unsigned int size, int flags) const
{
  if ((type_ == ZMQ_REQ) || (type_ == ZMQ_REP))
  {
    zmq::message_t msg(size);
    bool received = primary_->recv(&msg, flags);
    if(received)
    {
      if (size == msg.size())
      {
        memcpy(data, msg.data(), size);
        return true;
      }
      else
        std::cout << "Error: (ZeromqMessenger) Incomming message size " << msg.size() << " is different from expected size" << size << std::endl;
    }
    return false;
  }
  else if ((type_ == ZMQ_PUB) || (type_ == ZMQ_SUB))
  {
    if (buffer_size_ < size)
    {
      std::cout << "Error: (ZeromqMessenger) Buffer size is too small" << std::endl;
      return false;
    }

    if (flags & ZMQ_DONTWAIT)
    {
      mtx_->lock();
      memcpy(data, buffer_, size);
      mtx_->unlock();
    }
    else
    {
      mtx_->lock();
      *recv_update_ = 0;
      mtx_->unlock();

      struct timespec t0, t1;
      clock_gettime(CLOCK_MONOTONIC, &t0);
      double elapsed = 0;
      while (elapsed < 1.00)
      {
        mtx_->lock();
        if (*recv_update_)
        {
          memcpy(data, buffer_, size);
          mtx_->unlock();
//          std::cout << "State received within " << elapsed << " s" << std::endl;
          return true;
        }
        mtx_->unlock();

        usleep(500);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        elapsed = (t1.tv_sec - t0.tv_sec) + (static_cast<double>(t1.tv_nsec - t0.tv_nsec))/1.0e9;
      }

      // Take the last message if nothing was received
      mtx_->lock();
      memcpy(data, buffer_, size);
      mtx_->unlock();
      std::cout << "Zeromq timeout!" << std::endl;
    }
    return true;
  }
  return false;
}

ZeromqMessenger::~ZeromqMessenger()
{
  safe_delete(&mtx_);
  if (buffer_)
    delete[] buffer_;
  safe_delete(&recv_update_);
  zmq_close(syncService_);
  safe_delete(&syncService_);
  zmq_close(primary_);
  safe_delete(&primary_);
  safe_delete(&context_);
  safe_delete(&g_args);
}
