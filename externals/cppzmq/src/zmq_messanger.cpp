#include <zmq_messanger.h>
#include <iostream>

void ZeromqMessanger::init(const char* pub, const char* sub)
{
  // Prepare our context
  context_ = new zmq::context_t(1);

  // Prepare ZMQ publisher
  publisher_ = new zmq::socket_t(*context_, ZMQ_PUB);
  publisher_->bind(pub);

  // Prepare ZMQ subscriber
  subscriber_ = new zmq::socket_t(*this->context_, ZMQ_SUB);
  subscriber_->connect(sub);
  subscriber_->setsockopt(ZMQ_SUBSCRIBE, "", 0);
}

void ZeromqMessanger::send(const void* data, int size)
{
  zmq::message_t message(size);
  memcpy(message.data(), data, size);
  publisher_->send(message);
}

bool ZeromqMessanger::recv(void *data, int size, int flags)
{
  zmq::message_t update;
  bool received = subscriber_->recv(&update, flags);
  if(received)
    memcpy(data, update.data(), size);
  return received;
}
