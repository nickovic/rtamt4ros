#! /usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, SetModelState

freq = 10 #Hz
speed = 0.5 #m/s

class Agent(object):
    def __init__(self):
        rospy.wait_for_service ('/gazebo/get_model_state')
        self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_model_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.model = GetModelStateRequest()
        self.model.model_name='agent'

    def control_callback(self, event):
        result = self.get_model_srv(self.model)
        rospy.loginfo(result)

        state_msg = ModelState()
        state_msg.model_name = self.model.model_name
        state_msg.pose = result.pose
        state_msg.twist = result.twist
        state_msg.pose.position.x = result.pose.position.x + speed/freq
        resp = self.set_model_srv( state_msg )


if __name__ == '__main__':
    # Process arguments
    try:
        rospy.init_node('agent')
        agent = Agent()
        rospy.Timer(rospy.Duration(1.0/freq), agent.control_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
