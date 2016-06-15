#!/usr/bin/env python
# consumer
 
import json
import rospy
import zmq

import std_msgs.msg

addition_result = ''

def callback(data):
    """ Handle the return of adding information. """
    global addition_result

    addition_result = data.data

# Setup the contexts for communicating with the outside server. 
context = zmq.Context()
receiver = context.socket(zmq.PULL)
receiver.connect('tcp://127.0.0.1:5000')
sender = context.socket(zmq.PUSH)
sender.connect('tcp://127.0.0.1:5010')
 
# Setup the ROS topics for communicating with connected nodes.
rospy.init_node('transporter',anonymous=True)
pub = rospy.Publisher('adding_start', std_msgs.msg.Empty, queue_size=1)
sub = rospy.Subscriber('adding_result', std_msgs.msg.Float64, callback)

while True:
    # Get data off the pipe from the external source
    data = json.loads(receiver.recv())

    # Load the data into a parameter in ROS
    rospy.set_param('adding_data', data['genome'])

    # Send a ready message on the topic to the adder node
    pub.publish(std_msgs.msg.Empty())

    # Wait for a result to return from the adder node
    while addition_result == '':
        pass

    # Transmit the result back to the external source

    msg = json.dumps({'id':data['id'],'fitness':addition_result, 'ns':rospy.get_namespace(), 'name':rospy.get_name()})
    sender.send(msg)
    print (rospy.get_namespace(), addition_result)
    addition_result = ''
