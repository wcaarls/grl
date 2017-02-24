#ifndef ZEROMQ_MESSENGER_H
#define ZEROMQ_MESSENGER_H

#include <zmq.hpp>
#include <iostream>

//#define _REAL_LEO_

#ifdef _REAL_LEO_
#include <Mutex.hpp>
typedef CMutex zmq_mutex;
#else
#include <mutex>
typedef std::mutex zmq_mutex;
#endif


#define MSG_BUFFER_SIZE (60*sizeof(double))

class ZeromqMessenger
{
private:
  zmq::context_t      *context_;
  zmq::socket_t       *primary_;
  bool                connected_;
  int                 type_;
  zmq::socket_t       *syncService_;
  unsigned int        subscribers_, subscribers_expected_;

  // Worker thread
  pthread_t           worker_;
  zmq_mutex           *mtx_;
  char                *buffer_;
  unsigned int        buffer_size_;
  int                 *recv_update_;

public:
  ZeromqMessenger() : context_(NULL), primary_(NULL), syncService_(NULL), subscribers_(0), subscribers_expected_(1), mtx_(NULL), buffer_(NULL), buffer_size_(MSG_BUFFER_SIZE), recv_update_(NULL) {}
  ~ZeromqMessenger();

  void start(int type, const char *primaryAddr, const char *secondaryAddr = 0, const char *syncAddress = 0);
  bool send(const void *data, unsigned int size) const;
  bool recv(void* data, unsigned int size, int flags = 0) const;
  void sync();
  bool isConnected() const { return connected_; }
};

#endif // ZEROMQ_MESSENGER_H
