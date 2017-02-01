#ifndef ZEROMQ_MESSENGER_H
#define ZEROMQ_MESSENGER_H

#include <zmq.hpp>
#include <iostream>
#include <mutex>

#define ZMQ_SYNC_PUB 0x01
#define ZMQ_SYNC_SUB 0x02
#define ZMQ_SYNC_CLI 0x04

class ZeromqMessenger
{
private:
  zmq::context_t*     context_;
  zmq::socket_t*      publisher_;
  bool                connected_;
  int                 flags_;
  zmq::socket_t*      syncService_;

  // Worker thread
  pthread_t           worker_;
  std::mutex          *mtx_;
  char*               buffer_;
  int                 buffer_size_;

public:
  ZeromqMessenger() : syncService_(NULL), mtx_(NULL), buffer_(NULL), buffer_size_(60*sizeof(double)) {}
  ~ZeromqMessenger();

  void start(const char *pubAddress, const char *subAddress, const char *syncAddress = 0, int flags = 0);
  void send(const void *data, int size) const;
  bool recv(void* data, int size, int flags = 0) const;

};

#endif // ZEROMQ_MESSENGER_H
