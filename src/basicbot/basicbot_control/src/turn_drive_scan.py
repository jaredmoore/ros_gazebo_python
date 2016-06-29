#!/usr/bin/env python
"""
    Implements a basic controller that turns and drives towards an object.  

    Interacts with the world by reading in a rostopic "laser_scanner" that gives 
    sensor vision to the robot.

    Requires: 
        smach: sudo apt-get install ros-indigo-executive-smach
"""

import random
import rospy

from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetWorldProperties
from gazebo_msgs.msg import LinkStates

from world_step.srv import step_world 

class WorldStep(object):
    """ Connect to the step_world service and handle stepping the physics simulation. """

    def __init__(self):
        # Wait for the StepWorld node to start.
        rospy.wait_for_service('step_world')
        self.step = rospy.ServiceProxy('step_world',step_world)
    
    def stepPhysics(self,steps=1):
        """ Step the simulation. """
        self.step()


###########################

class GetLinkStates(object):
    """ Connect to the /gazebo/link_states topic and get information about each link. 
        Message has three lists: name, pose, twist
    """

    def __init__(self):
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.lsCallback)

        self.msg = ""
        self.formatted_msg = {}

    def lsCallback(self,msg):
        """ Callback for the link_states topic. """
        self.msg = msg
        for n,p,t in zip(msg.name,msg.pose,msg.twist):
            self.formatted_msg[n] = {'pose':p,'twist':t}

    def getStates(self):
        """ Return the last received message. """
        if self.msg:
            return type(self.msg.pose)
        else:
            return ""
    
    def getLinkPose(self,link):
        """ Return the pose of a specific link from the last message. """
        if link in self.formatted_msg:
            return self.formatted_msg[link]['pose']
        else:
            return ""

###########################

class GetLaserScanner(object):
    """ Connect to /laser_scanner rostopic and get information about the sensor.
    Message format is: 
    """

    def __init__(self):
        rospy.Subscriber('/laser_scanner', LaserScan, self.lsCallback)

        self.msg = ""
        self.formatted_msg = ""

    def lsCallback(self,msg):
        """ Callback for the laser_scanner topic. """
        self.msg = msg
        self.formatted_msg = {'time':str(msg.header.stamp.secs)+"."+str(msg.header.stamp.nsecs), 'sum_ranges':sum(msg.ranges), 'ranges':msg.ranges}

    def getScanState(self):
        """ Get the current scan information. """
        return self.formatted_msg
        
    def getLeftCenterRightScanState(self):
        """ Divide the vision into three sections and report on their average sum. """

        # If no message yet return blank
        if not self.formatted_msg:
            return []

        partitioned_vision = []

        partitions = [len(self.formatted_msg['ranges'])/3 for i in range(3)]

        # Add additional ones to middle if don't match sum.
        if sum(partitions) < len(self.formatted_msg['ranges']):
            partitions[1] += len(self.formatted_msg['ranges']) - sum(partitions)

        # Calculate the index offsets.
        partitions[1] = partitions[0] + partitions[1]
        partitions[2] = partitions[1] + partitions[2]

        # Get right, center, and left averages.
        partitioned_vision.append(sum(self.formatted_msg['ranges'][0:partitions[0]])/len(self.formatted_msg['ranges'][0:partitions[0]]))
        partitioned_vision.append(sum(self.formatted_msg['ranges'][partitions[0]:partitions[1]])/len(self.formatted_msg['ranges'][partitions[0]:partitions[1]]))
        partitioned_vision.append(sum(self.formatted_msg['ranges'][partitions[1]:partitions[2]])/len(self.formatted_msg['ranges'][partitions[1]:partitions[2]]))

        return partitioned_vision

###########################

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
    cmd_vel_pub.publish(twist['movement'])

###########################

import smach
import smach_ros

# Define the states for the robot

class SpinLeft(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['spin_left','drive_forward'],
            input_keys=['detect_center'])

    def execute(self, userdata):
        MoveRobot('left')
        if userdata.detect_center < 10.0:
            return 'drive_forward'
        else: 
            return 'spin_left'

class SpinRight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['spin_right','drive_forward'],
            input_keys=['detect_center'])

    def execute(self, userdata):
        MoveRobot('right')
        if userdata.detect_center < 10.0:
            return 'drive_forward'
        else: 
            return 'spin_right'

class DriveForward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['spin_right','drive_forward','spin_left','within_threshold'],
            input_keys=['detect_center','detect_right','detect_left'])
        self.threshold = 8.0

    def execute(self, userdata):
        MoveRobot('forward')
        if userdata.detect_center < self.threshold:
            return 'within_threshold'
        elif userdata.detect_center < 10.0: 
            return 'drive_forward'
        elif userdata.detect_leftt < 10.0 and userdata.detect_center >= 10.0 and userdata.detect_right >= 10.0: 
            return 'spin_left'
        else:
            return 'spin_right'

class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','spin_right'],
            input_keys=['detect_center'])

    def execute(self, userdata):
        MoveRobot('stop')
        if userdata.detect_center < 10.0:
            return 'succeeded'
        else:
            return 'spin_right'

###########################

# Setup the reset world and reset simulation services
rospy.wait_for_service('/gazebo/get_world_properties')
rospy.wait_for_service('/gazebo/reset_world')
rospy.wait_for_service('/gazebo/reset_simulation')
rospy.wait_for_service('/gazebo/pause_physics')
rospy.wait_for_service('/gazebo/unpause_physics')

getWorldProp = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
resetWorld = rospy.ServiceProxy('/gazebo/reset_world', Empty)
resetSimulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

ws = WorldStep()
# Setup the messages we will use to drive the robot.
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('wander')

# Setup the topic subscribers.
ls = GetLinkStates()
scan = GetLaserScanner()

sm = smach.StateMachine(outcomes=['succeeded'])

with sm:
        smach.StateMachine.add('SPIN_RIGHT', SpinRight(), transitions={ 'spin_right':'SPIN_RIGHT',
            'drive_forward':'DRIVE_FORWARD'
            })
        smach.StateMachine.add('SPIN_LEFT', SpinLeft(), transitions={ 'spin_left':'SPIN_LEFT',
            'drive_forward':'DRIVE_FORWARD'
            })
        smach.StateMachine.add('DRIVE_FORWARD', DriveForward(), transitions={ 'spin_right':'SPIN_RIGHT',
            'drive_forward':'DRIVE_FORWARD',
            'spin_left':'SPIN_LEFT',
            'within_threshold':'STOP'
            })
        smach.StateMachine.add('STOP', Stop(), transitions={ 'succeeded':'succeeded', 'spin_right':'SPIN_RIGHT'})

        outcome = sm.execute()

# Set the initial state of the robot.
driving_forward = False
state_change_time = getWorldProp().sim_time 

# Start the simulation with an initial step.
ws.stepPhysics(steps=1)
current_time = getWorldProp().sim_time 
final_time = getWorldProp().sim_time + 10.0
print("Final Time: "+str(final_time)+", Current Time: "+str(current_time))
ws.stepPhysics(steps=1)

while not rospy.is_shutdown():
    ws.stepPhysics(steps=1)
    print(scan.getLeftCenterRightScanState())
    # if driving_forward:
    #     cmd_vel_pub.publish(turn_twist)
    # else:
    #     cmd_vel_pub.publish(wander_twist)
    if state_change_time < getWorldProp().sim_time:
        # driving_forward = not driving_forward
        # turn_twist.angular.z = turn_twist.angular.z# * random.choice([-1,1])
        state_change_time = getWorldProp().sim_time + 2.5 
        print(str(getWorldProp().sim_time)+","+str(ls.getLinkPose('basicbot::base_link').position.x)+","+str(ls.getLinkPose('basicbot::base_link').position.y))
    if final_time <= getWorldProp().sim_time:
        break

print(str(getWorldProp().sim_time)+","+str(ls.getLinkPose('basicbot::base_link').position.x)+","+str(ls.getLinkPose('basicbot::base_link').position.y))

