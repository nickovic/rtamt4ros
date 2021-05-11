#! /usr/bin/env python
# call roscore
# $ roscore
#
# IF start in manual
# $ rosrun hsr_monitors wrong_global_path.py

import rospy

#other msg
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from tmc_omni_path_follower.msg import PathFollowerActionGoal


class WrongGlobalPath(object):
    def __init__(self):
        self.wrongGlobalPath_publisher = rospy.Publisher('/path_follow_action/goal', PathFollowerActionGoal, queue_size=10)
        self.wrongGlobalPath_view_publisher = rospy.Publisher('/base_local_path', Path, queue_size=10)


    def main(self):
        header = Header()
        header.seq = 0
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        start_poseStamped = PoseStamped()
        start_poseStamped.header = header
        start_poseStamped.pose.position.x = 0.1
        start_poseStamped.pose.position.y = 0.0
        start_poseStamped.pose.position.z = 0.0
        start_poseStamped.pose.orientation.x = 0.0
        start_poseStamped.pose.orientation.y = 0.0
        start_poseStamped.pose.orientation.z = 0.0
        start_poseStamped.pose.orientation.w = 1.0

        end_poseStamped = PoseStamped()
        end_poseStamped.header = header
        end_poseStamped.pose.position.x = 6.0
        end_poseStamped.pose.position.y = 3.0
        end_poseStamped.pose.position.z = 0.0
        end_poseStamped.pose.orientation.x = 0.0
        end_poseStamped.pose.orientation.y = 0.0
        end_poseStamped.pose.orientation.z = 0.7
        end_poseStamped.pose.orientation.w = 0.7

        path = Path()
        path.header = header
        path.poses = [start_poseStamped, end_poseStamped]

        pathFollowerActionGoal = PathFollowerActionGoal()
        pathFollowerActionGoal.header = header
        pathFollowerActionGoal.goal.path = path

        for i in range(5):
            # wrong path actuial order
            self.wrongGlobalPath_publisher.publish(pathFollowerActionGoal)
            # wrong path for viewer
            self.wrongGlobalPath_view_publisher.publish(path)
            rospy.sleep(0.1)


if __name__ == '__main__':

	try:
		rospy.init_node('wrong_global_path')
		wrongGlobalPath = WrongGlobalPath()
		wrongGlobalPath.main()

	except rospy.ROSInterruptException:
		pass
