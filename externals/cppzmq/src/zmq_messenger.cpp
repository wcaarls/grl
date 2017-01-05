#include <zmq_messenger.h>
#include <iostream>
#include <thread>         // std::thread
#include <mutex>          // std::mutex

std::string gSubAddress;
std::mutex gMtx;
const int gSize = 20*sizeof(double);
char gData[gSize];

void *worker_routine (void *context)
{
  // Prepare ZMQ subscriber
  int confl = 1;
  zmq::socket_t* subscriber = new zmq::socket_t(*(zmq::context_t*)context, ZMQ_SUB);
  subscriber->connect(gSubAddress.c_str());
  subscriber->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl)); // Keep only last message
  subscriber->setsockopt(ZMQ_SUBSCRIBE, "", 0);

  while (1)
  {
    zmq::message_t update;
    bool received = subscriber->recv(&update, ZMQ_DONTWAIT);
    if(received)
    {
      gMtx.lock();
      memcpy(gData, update.data(), gSize);
      gMtx.unlock();
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

  // Prepare ZMQ publisher
  publisher_ = new zmq::socket_t(*context_, ZMQ_PUB);
  publisher_->bind(pubAddress);
  publisher_->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl)); // Keep only last message

  gSubAddress = std::string(subAddress);
  pthread_create (&worker_, NULL, worker_routine, context_);

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
  assert(gSize == size);
  gMtx.lock();
  memcpy(data, gData, size);
  gMtx.unlock();
  return true;
}
