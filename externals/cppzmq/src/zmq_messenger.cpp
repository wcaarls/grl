#include <zmq_messenger.h>
#include <thread>
#include <unistd.h>

struct worker_args {
    zmq::context_t* context;
    const char*     subAddress;
    std::mutex*     mtx;
    char*           buffer;
    int             buffer_size;
    int             *recv_update;
};

worker_args         g_args;

void *worker_routine(void *param)
{
  // Prepare ZMQ subscriber
  int confl = 1;
  worker_args *args = (worker_args*) param;
  zmq::socket_t* subscriber = new zmq::socket_t(*(args->context), ZMQ_SUB);
  subscriber->connect(args->subAddress);
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
  return NULL;
}

void ZeromqMessenger::start(const char* pubAddress, const char* subAddress, const char* syncAddress, int flags)
{
  flags_ = flags;
  int confl = 1;

  // Prepare our context
  context_ = new zmq::context_t(1);

  if (flags_ & ZMQ_SYNC_CLI)
  {
    publisher_ = new zmq::socket_t(*context_, ZMQ_REQ);
    publisher_->connect(pubAddress);
  }
  else
  {
    // Prepare ZMQ publisher
    publisher_ = new zmq::socket_t(*context_, ZMQ_PUB);
    publisher_->bind(pubAddress);
    publisher_->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl)); // Keep only last message

    // Prepare ZMQ subscriber
    mtx_ = new std::mutex();
    buffer_ = new char [buffer_size_];
    recv_update_ = new int(0);
    g_args = {context_, subAddress, mtx_, buffer_, buffer_size_, recv_update_};
    int ret = pthread_create(&worker_, NULL, worker_routine, (void*)&g_args);
    if (ret)
    {
      std::cout << "Failed creating zmq worker thread with error " << ret << std::endl;
      abort();
    }

    if (flags_ & ZMQ_SYNC_PUB)
    {
      syncService_ = new zmq::socket_t(*context_, ZMQ_REP);
      syncService_->bind(syncAddress);
    }

    if (flags_ & ZMQ_SYNC_SUB)
    {
      //std::cout << "Trying to connect" << std::endl;

      // synchronize with publisher
      syncService_ = new zmq::socket_t(*context_, ZMQ_REQ);
      syncService_->connect(syncAddress);

      // - send a synchronization request
      zmq::message_t message(0);
      syncService_->send(message);

      // - wait for synchronization reply
      zmq::message_t update;
      syncService_->recv(&update);

      // Third, get our updates and report how many we got
      //std::cout << "Ready to receive" << std::endl;
      zmq_close(syncService_);
     }
  }
}

void ZeromqMessenger::send(const void* data, int size) const
{
  zmq::message_t message(size);
  memcpy(message.data(), data, size);
  publisher_->send(message);
}

bool ZeromqMessenger::recv(void *data, int size, int flags) const
{
  if (flags_ & ZMQ_SYNC_CLI)
  {
    zmq::message_t reply(size);
    publisher_->recv(&reply);
    memcpy(data, reply.data(), size);
    return true;
  }
  else
  {
    if (buffer_size_ < size)
    {
      std::cout << "Error: (ZeromqMessenger) Buffer size is too small" << std::endl;
      return false;
    }

    if (flags & ZMQ_NOBLOCK)
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
      while (elapsed < 0.060)
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
}

ZeromqMessenger::~ZeromqMessenger()
{
  if (mtx_)
    delete mtx_;
  if (buffer_)
    delete[] buffer_;
  if (recv_update_)
    delete recv_update_;
}
