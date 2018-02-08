import threading, socket

class Server():
  def __init__(self, port=3373):
    """Spawn thread to listen to connections"""
    self.port = port
    self.workers = []
    self.condition = threading.Condition()
    self.thread = threading.Thread(target=Server.thread, args=[self])
    self.thread.daemon = True
    self.thread.start()

  def thread(self):
    """Listen to connections, adding them to available workers"""
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', self.port))
    s.listen(100)
    
    print "Server listening for connections on port", self.port
    
    try:
      while True:
        c, _ = s.accept()
        with self.condition:
          self.workers.append(c)
          self.condition.notify()
    except e:
      raise e

  def submit(self, conf):
    """ Submit job to available worker"""
    # Wait for available worker
    with self.condition:
      while len(self.workers) == 0:
        self.condition.wait()
      w = self.workers.pop()
    
    # Send configuration
    w.send(conf + '\0')
    
    # Return stream for reading by submitter
    return w

def read(w, regret):
  """Read worker result, returning either simple or cumulative regret"""
  data = [float(v) for v in w.makefile().readlines()]
  w.shutdown(socket.SHUT_RDWR)
  w.close()
  
  if regret == 'simple':
    sample = int(len(data)/20)
    return sum(data[-sample:])/sample
  elif regret == 'cumulative':
    return sum(data)
  else:
    raise "Unknown regret type", regret

if __name__ == '__main__':
  server = Server(3373)
  
  with open('server.yaml', 'r') as f:
    conf = f.read()
  
  w = server.submit(conf)
  w2 = server.submit(conf)
  
  print read(w)
  print read(w2)
