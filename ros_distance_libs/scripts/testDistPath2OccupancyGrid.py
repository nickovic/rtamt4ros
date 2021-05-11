#! /usr/bin/env python
# call roscore
# $ roscore
#
# IF start in manual
# $ rosrun ros_distance_libs testDistPath2OccupancyGrid.py

import matplotlib.pyplot as plt
import timeit

import rospy

from ros_distance_libs.rosDistLib import *

#other msg
from nav_msgs.msg import OccupancyGrid, Path


class TestDistPath2OccupancyGrid(object):

	def __init__(self):
		rospy.Subscriber('/static_distance_map_ref', OccupancyGrid, self.map_callback, queue_size=10)
		self.map = []

		rospy.Subscriber('/base_local_path', Path, self.globalPath_callback, queue_size=10)
		self.globalPath = []


	def map_callback(self, occupancyGrid):
		self.map = occupancyGrid


	def globalPath_callback(self, path):
		self.globalPath = path


	def main(self):
		rospy.loginfo('waiting data')

		while not rospy.is_shutdown():
			if self.map != [] and self.globalPath != []:
				break

		occupancyGrid = self.map
		path = self.globalPath

		# internal
		t_start = timeit.default_timer()

		mapPoints = occupancyGridData2PointList(occupancyGrid)
		rospy.loginfo('#point : {}'.format(mapPoints.size))
		pathList = [(iPoseStamped.pose.position.x, iPoseStamped.pose.position.y) for iPoseStamped in path.poses]
		rospy.loginfo('#path : {}'.format(len(pathList)))
		t_convsrsion = timeit.default_timer()

		dist = distMultiPoint2LineString(mapPoints, pathList)
		t_shape = timeit.default_timer()
		rospy.loginfo('Dist: {}'.format(dist))

		rospy.loginfo('Computation time[s]: conversion={:0.8f}, shape={:0.8f}'.format(t_convsrsion-t_start, t_shape-t_convsrsion))

		# distPath2OccupancyGrid
		t_start = timeit.default_timer()
		dist = distPath2OccupancyGrid(path, occupancyGrid)
		t_dist = timeit.default_timer()
		rospy.loginfo('distPath2OccupancyGrid time[s]: {:0.8f}'.format(t_dist-t_start))

		mapFig = plt.figure(figsize = (12,12))
		ax = mapFig.add_subplot(1, 1, 1)
		x = [iPoint[0] for iPoint in mapPoints]
		y = [iPoint[1] for iPoint in mapPoints]
		ax.scatter(x, y)
		x = [iPoint[0] for iPoint in pathList]
		y = [iPoint[1] for iPoint in pathList]
		ax.plot(x, y)
		plt.show()


if __name__ == '__main__':

	try:
		rospy.init_node('hsr_stl_monitor')
		testDistPath2OccupancyGrid = TestDistPath2OccupancyGrid()
		testDistPath2OccupancyGrid.main()

	except rospy.ROSInterruptException:
		pass
