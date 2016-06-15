# producer
 
import json
import zmq
 
import random
 
import threading
 
class SenderThread(threading.Thread):
    def __init__(self, threadID, socket, genomes):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.socket = socket
        self.data = genomes
 
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
        for i in self.data:
            msg = json.dumps(i)
            print(msg)
            self.socket.send(msg)
 
class GACommunicator(object):
    """ Class to handle setting up the sockets and sending/receiving genome data. """

    def __init__(self):
        """ Initialize the socket for data. """

        # Setup the socket to send data out on.
        context = zmq.Context()
        self.socket = context.socket(zmq.PUSH)
        #socket.setsockopt(zmq.LINGER, 0)    # discard unsent messages on close
        self.socket.bind('tcp://127.0.0.1:5000')
 
        # Setup the socket to read the responses on.
        self.receiver = context.socket(zmq.PULL)
        self.receiver.bind('tcp://127.0.0.1:5010')

    def __del__(self):
        """ Close the sockets. """
        print("Closing Socket")
        self.socket.close()
        self.receiver.close()

    def send_genomes(self,genomes):
        """ Send the genomes through the sender thread to workers. 

        Args:
            genomes: list of genomes to send out.

        Returns:
            list of results containing the genome id and fitness
        """

        return_data = []

        # Start a thread to send the data.
        sendThread = SenderThread(1, self.socket, genomes)

        sendThread.start()
 
        # Read the responses on the receiver socket.
        i = len(genomes)
        while i > 0:
            data = json.loads(self.receiver.recv())
            #return_data.append({'id':data['id'], 'fitness':data['fitness']})
            return_data.append({'sum':data['sum'],'true_sum':data['a']+data['b']})
            i -= 1
         
        # Wait for the send thread to complete.
        sendThread.join()

        return return_data

ga_communicator = GACommunicator()

for i in range(5):
    print("Sending tasks to workers")
    genomes = [{'a':random.random(),'b':random.random()*10.0} for i in range(10)]
    print(ga_communicator.send_genomes(genomes))
    print("\n\n\n\n")
 
# TODO: Auto-detect when workers are ready?
# TODO: Change messages so they persist waiting for workers?
#print("Press Enter when the workers are ready: ")
#_ = raw_input()



