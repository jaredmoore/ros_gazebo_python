#!/usr/bin/env python
"""
    Implements a basic controller that turns and drives towards an object 
    without a state machine.  Essentially it drives only based on a certain
    clock time.

"""

import random
import rospy
import std_msgs.msg

from std_srvs.srv import Empty

from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetWorldProperties

from basicbot_utils import GetLaserScanner, GetLinkStates
from world_step import WorldStep

###########################

final_time = 0.0

# Setup the driving messages.
twist = {}
twist['forward'] = Twist()
twist['forward'].linear.x = 0.5

twist['stop'] = Twist()
twist['stop'].linear.x = 0.0

twist['left'] = Twist()
twist['left'].linear.x = 0.0
twist['left'].angular.z = -0.5

twist['right'] = Twist()
twist['right'].linear.x = 0.0
twist['right'].angular.z = 0.5

def MoveRobot(movement):
    """ Movements: 'left', 'forward', 'right', 'stop' """
    cmd_vel_pub.publish(twist[movement])

def checkAtFinalTime():
    """ Check to see if we have exceeded the execution time. """
    global final_time
    if final_time <= getWorldProp().sim_time:
        return True
    return False


def simCallback(data):
    """ Callback to conduct a simulation. """
    global final_time, pub, bot_position, bot_id

    genome_data = rospy.get_param('basicbot_genome')

    # Set the first timestep
    ws.stepPhysics(steps=1)
    current_time = getWorldProp().sim_time 
    final_time = getWorldProp().sim_time + 10.0

    print("Starting an individual simulation.")
    print(str(getWorldProp().sim_time)+","+str(ls.getLinkPose('basicbot::base_link').position.x)+","+str(ls.getLinkPose('basicbot::base_link').position.y))
    print("--------------------------")

    while not checkAtFinalTime():
        ws.stepPhysics(steps=1)
        current_time = getWorldProp().sim_time 

        if current_time >= 5.0 and current_time < 5.2:
            MoveRobot('left')
        else:
            MoveRobot('forward')

    current_time = getWorldProp().sim_time 

    print("Ending a simulation:")
    print("Current Time: "+str(current_time))
    print(str(getWorldProp().sim_time)+","+str(ls.getLinkPose('basicbot::base_link').position.x)+","+str(ls.getLinkPose('basicbot::base_link').position.y))
    print("--------------------------")

    # Publish the resulting time on the topic.
    pub.publish(current_time)

    resetWorld()
    resetSimulation()

###########################

# Initial setup of the node, required services, etc.

ns = rospy.get_namespace()
print("Namespace: ",ns)
# Setup the reset world and reset simulation services
rospy.wait_for_service(ns+'/gazebo/get_world_properties')
rospy.wait_for_service(ns+'/gazebo/reset_world')
rospy.wait_for_service(ns+'/gazebo/reset_simulation')
rospy.wait_for_service(ns+'/gazebo/pause_physics')
rospy.wait_for_service(ns+'/gazebo/unpause_physics')

getWorldProp = rospy.ServiceProxy(ns+'/gazebo/get_world_properties', GetWorldProperties)
resetWorld = rospy.ServiceProxy(ns+'/gazebo/reset_world', Empty)
resetSimulation = rospy.ServiceProxy(ns+'/gazebo/reset_simulation', Empty)

# Setup the WorldStep service object
ws = WorldStep()

# Setup the messages publisher we will use to drive the robot.
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# Initialize the node.
rospy.init_node('turn_drive_scan_node', log_level=rospy.WARN, anonymous=True)

# Setup the topic subscribers for getting the state of the robot and sensors.
ls = GetLinkStates()

# Setup the callbacks for starting and reporting results.
sub = rospy.Subscriber('simulation_start', std_msgs.msg.Empty, simCallback)
pub = rospy.Publisher('simulation_result', std_msgs.msg.Float64, queue_size=1)

# Spin the node and wait for the callback.
rospy.spin()
