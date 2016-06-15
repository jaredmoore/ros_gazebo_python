# producer
 
import json
import zmq
 
import random
 
import threading
 
class senderThread(threading.Thread):
    def __init__(self, threadID, socket):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.socket = socket
 
    def run(self):
        print("\t\t\t\tStarting "+str(self.threadID))
        self.send_data()
        print("\t\t\t\tExiting "+str(self.threadID))
 
    def send_data(self):
        """ Send data to worker processes. 
 
        Args:
            socket: socket to send the data out on.
                - Persistant throughout execution for now.
        """
        for i in range(10):
            msg = json.dumps({'a':random.random(),'b':random.random()*10.0})
            print(msg)
            socket.send(msg)
 
# Setup the socket to send data out on.
context = zmq.Context()
socket = context.socket(zmq.PUSH)
#socket.setsockopt(zmq.LINGER, 0)    # discard unsent messages on close
socket.bind('tcp://127.0.0.1:5000')
 
# Setup the socket to read the responses on.
receiver = context.socket(zmq.PULL)
receiver.bind('tcp://127.0.0.1:5010')
 
print("Press Enter when the workers are ready: ")
_ = raw_input()
print("Sending tasks to workers")
 
# Start a thread to send the data.
sendThread = senderThread(1, socket)
sendThread.start()
 
# Read the responses on the receiver socket.
i = 10
while i > 0:
    data = json.loads(receiver.recv())
    print(data['sum'],data['a']+data['b'])
    i -= 1
 
# Wait for the send thread to complete.
sendThread.join()
 
# while threading.activeCount() > 0:
#     #print(threading.activeCount())
#     pass
 
print("Closing Socket")
socket.close()
receiver.close()
