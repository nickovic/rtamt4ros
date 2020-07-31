#! /usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, SetModelState

rospy.init_node('agent')
rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
set_model_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

odom=Odometry()
header = Header()
header.frame_id='/map'

model = GetModelStateRequest()
model.model_name='unit_cylinder'

r = rospy.Rate(1)

while not rospy.is_shutdown():
    result = get_model_srv(model)
    rospy.loginfo(result)

    state_msg = ModelState()
    state_msg.model_name = model.model_name
    state_msg.pose = result.pose
    state_msg.twist = result.twist
    state_msg.pose.position.x = result.pose.position.x + 0.1
    resp = set_model_srv( state_msg )

    r.sleep()
