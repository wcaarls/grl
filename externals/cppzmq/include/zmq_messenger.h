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
  //zmq::socket_t*      subscriber_;
  bool                connected_;
  int                 flags_;
  zmq::socket_t*      syncService_;
  int                 subscribers_expected_;
  int                 subscribers_;
  pthread_t           subscriber_worker_;

public:
  ZeromqMessenger() : connected_(false), subscribers_(0), subscribers_expected_(1), syncService_(NULL) {}

  void init(const char *pubAddress, const char *subAddress, const char *syncAddress = 0, int flags = 0);
  void sync();
  void send(const void *data, int size) const;
  bool isConnected() const { return connected_; }
  bool recv(void* data, int size, int flags = 0) const;

};

#endif // ZEROMQ_MESSENGER_H
