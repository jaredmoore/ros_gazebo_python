import rospy

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