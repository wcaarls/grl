#ifndef ZEROMQ_MESSENGER_H
#define ZEROMQ_MESSENGER_H

#include <zmq.hpp>

#define ZMQ_SYNC_PUB 0x01
#define ZMQ_SYNC_SUB 0x02

class ZeromqMessenger
{
private:
  zmq::context_t*     context_;
  zmq::socket_t*      publisher_;
  zmq::socket_t*      subscriber_;

public:
  ZeromqMessenger() {}

  void init(const char *pub, const char *sub, const char *sync = 0, int flags = 0);
  void send(const void *data, int size) const;
  bool recv(void* data, int size, int flags = 0) const;
};

#endif // ZEROMQ_MESSENGER_H
