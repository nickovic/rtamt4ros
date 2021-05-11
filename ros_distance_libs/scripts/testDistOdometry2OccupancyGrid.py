#! /usr/bin/env python
# call roscore
# $ roscore
#
# IF start in manual
# $ rosrun ros_distance_libs testDistOdometry2OccupancyGrid.py

import timeit

import rospy

from ros_distance_libs.rosDistLib import *

#other msg
from nav_msgs.msg import Odometry, OccupancyGrid


class TestDistOdometry2OccupancyGrid(object):

	def __init__(self):
		rospy.Subscriber('/dynamic_obstacle_map', OccupancyGrid, self.dynamicObsMap_callback, queue_size=10)
		self.dynamicObsMap = []
		rospy.Subscriber('/hsrb/odom_ground_truth', Odometry, self.odom_gt_callback, queue_size=10)
		self.loc_gt = []


	def dynamicObsMap_callback(self, occupancyGrid):
		self.dynamicObsMap = occupancyGrid


	def odom_gt_callback(self, odometry):
		self.loc_gt = odometry


	def main(self):
		rospy.loginfo('waiting data')

		while not rospy.is_shutdown():
			if self.dynamicObsMap != [] and self.loc_gt != []:
				break

		occupancyGrid = self.dynamicObsMap
		odometry = self.loc_gt


		# distPath2OccupancyGrid
		t_start = timeit.default_timer()
		dist, stamp = distOdometry2OccupancyGrid(odometry, occupancyGrid, 3)
		t_dist = timeit.default_timer()
		rospy.loginfo('Dist: {}'.format(dist))
		rospy.loginfo('distOdometry2OccupancyGrid time[s]: {:0.8f}'.format(t_dist-t_start))


if __name__ == '__main__':

	try:
		rospy.init_node('hsr_stl_monitor')
		testDistOdometry2OccupancyGrid = TestDistOdometry2OccupancyGrid()
		testDistOdometry2OccupancyGrid.main()

	except rospy.ROSInterruptException:
		pass
