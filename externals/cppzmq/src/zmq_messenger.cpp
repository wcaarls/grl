#include <zmq_messenger.h>
#include <iostream>

void ZeromqMessenger::init(const char* pub, const char* sub, const char* sync, int flags)
{
  // Prepare our context
  context_ = new zmq::context_t(1);

  // Prepare ZMQ publisher
  publisher_ = new zmq::socket_t(*context_, ZMQ_PUB);
  publisher_->bind(pub);

  // Prepare ZMQ subscriber
  subscriber_ = new zmq::socket_t(*this->context_, ZMQ_SUB);
  subscriber_->connect(sub);
  int confl = 1;
  subscriber_->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl)); // Keep only last message
  subscriber_->setsockopt(ZMQ_SUBSCRIBE, "", 0);

  if (flags & ZMQ_SYNC_PUB)
  {
    zmq::socket_t* syncService = new zmq::socket_t(*context_, ZMQ_REP);
    syncService->bind(sync);

    std::cout << "Waiting for subscribers" << std::endl;
    int subscribers_expected = 1;
    int subscribers = 0;
    while (subscribers < subscribers_expected)
    {
      // - wait for synchronization request
      zmq::message_t update;
      syncService->recv(&update);

      // - send synchronization reply
      zmq::message_t message(0);
      syncService->send(message);

      subscribers++;
    }
    std::cout << subscribers << " subscriber(s) connected" << std::endl;
  }

  if (flags & ZMQ_SYNC_SUB)
  {
    std::cout << "Trying to connect" << std::endl;

    // synchronize with publisher
    zmq::socket_t* syncService = new zmq::socket_t(*context_, ZMQ_REQ);
    syncService->connect(sync);

    // - send a synchronization request
    zmq::message_t message(0);
    syncService->send(message);

    // - wait for synchronization reply
    zmq::message_t update;
    syncService->recv(&update);

    // Third, get our updates and report how many we got
    std::cout << "Ready to receive" << std::endl;
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
