#!/usr/bin/python
import zmq
import time
import sys
import struct

READ     = 0
WRITE    = 1
UPDATE   = 2
FINALIZE = 3

port = "5556"
if len(sys.argv) > 1:
    port =  sys.argv[1]
    int(port)
    
context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:%s" % port)

mem = dict()

print "WARNING: THIS TEST REPRESENTATION ONLY SUPPORTS A ONE-DIMENSIONAL OUTPUT"

while True:
    # Wait for next request from client
    message = socket.recv()
    
    #print "Got message, length", len(message)
    
    tp = struct.unpack("d", message[0:8])[0]
    message = message[8:]
    
    #print "Message type", tp
    
    if tp == READ:
      # Read
      #print "Read"
      
      sz = len(message) / 8
      #print "Input has", sz, "elements"
      
      x = struct.unpack(str(sz) + "d", message)
      #print "Input", x
      
      if x in mem:
        y = mem[x]
      else:
        y = 0

      #print "Output", y
      
      socket.send(struct.pack("d", y))
    elif tp == WRITE:
      # Write
      #print "Write"
      
      sz = (len(message)-16) / 8
      #print "Input has", sz, "elements"
      
      vec = struct.unpack(str(sz) + "ddd", message)
      x = vec[0:sz]
      y = vec[sz]
      a = vec[sz+1]
      
      #print "Input", x
      #print "Output", y
      #print "Alpha", a
      
      if x in mem:
        mem[x] = (1-a) * mem[x] + a * y
      else:
        mem[x] = a * y
      
      socket.send(str())
    elif tp == UPDATE:
      # Update
      #print "Update"
      
      sz = (len(message)-8) / 8
      #print "Input has", sz, "elements"
      
      vec = struct.unpack(str(sz) + "dd", message)
      x = vec[0:sz]
      d = vec[sz]
      
      #print "Input", x
      #print "Delta", d
      
      if x in mem:
        mem[x] = mem[x] + d
      else:
        mem[x] = d
      
      socket.send(str())
    else:
      socket.send(str())
    