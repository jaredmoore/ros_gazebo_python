#!/usr/bin/env python

import rospy
import std_msgs.msg

def addCallback(data):
    """ Add the two numbers provided in the ROS Param 'adding_data' """
    add_data = rospy.get_param('adding_data')

    pub.publish(sum(add_data))

rospy.init_node('adder',anonymous=True)
sub = rospy.Subscriber('adding_start', std_msgs.msg.Empty, addCallback)
pub = rospy.Publisher('adding_result', std_msgs.msg.Float64, queue_size=1)
rospy.spin()
