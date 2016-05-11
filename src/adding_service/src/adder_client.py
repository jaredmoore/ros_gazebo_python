#!/usr/bin/env python
# consumer
 
import json
import rospy
import zmq
 
context = zmq.Context()
receiver = context.socket(zmq.PULL)
receiver.connect('tcp://127.0.0.1:5000')
 
sender = context.socket(zmq.PUSH)
sender.connect('tcp://127.0.0.1:5010')
 
while True:
    data = json.loads(receiver.recv())
    msg = json.dumps({'a':data['a'],'b':data['b'],'sum':data['a']+data['b'], 'ns':rospy.get_namespace(), 'name':rospy.get_name()})
    sender.send(msg)
    print (rospy.get_namespace(), data['a']+data['b'])
