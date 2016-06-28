#include <zmq_messenger.h>
#include <iostream>

void ZeromqMessenger::init(const char* pubAddress, const char* subAddress, const char* syncAddress, int flags)
{
  flags_ = flags;
  int confl = 1;

  // Prepare our context
  context_ = new zmq::context_t(1);

  // Prepare ZMQ publisher
  publisher_ = new zmq::socket_t(*context_, ZMQ_PUB);
  publisher_->bind(pubAddress);
  publisher_->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl)); // Keep only last message

  // Prepare ZMQ subscriber
  subscriber_ = new zmq::socket_t(*this->context_, ZMQ_SUB);
  subscriber_->connect(subAddress);
  subscriber_->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl)); // Keep only last message
  subscriber_->setsockopt(ZMQ_SUBSCRIBE, "", 0);

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

void ZeromqMessenger::sync()
{
  //std::cout << "sync" << std::endl;
  if (connected_)
    return;

  if (flags_ & ZMQ_SYNC_PUB)
  {
    //std::cout << "Waiting for subscribers" << std::endl;
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

    //std::cout << subscribers_ << " subscriber(s) connected" << std::endl;
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
  zmq::message_t update;
  bool received = subscriber_->recv(&update, flags);
  if(received)
    memcpy(data, update.data(), size);
  return received;
}
