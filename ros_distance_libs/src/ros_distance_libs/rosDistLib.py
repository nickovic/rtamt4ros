# TODO:
# 1) distsPointCloud2OccupancyGrid, distsOdometry2OccupancyGrid are too slow.
# 2) support 3D

import rospy
import numpy
import tf
import sensor_msgs.point_cloud2

from std_msgs.msg import String
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2, PointCloud, LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point32, Vector3

from shapelyLib import *

import matplotlib.pyplot as plt


# ROS data conversion -----
def orientation2angular(orientation):
	quaternion = (  orientation.x,
					orientation.y,
					orientation.z,
					orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	angular = Vector3(
		euler[0],
		euler[1],
		euler[2]
	)
	return angular


def pointCloud22PointCloud(pointCloud2):
	points_gen = sensor_msgs.point_cloud2.read_points(pointCloud2)
	points_list = []
	for point in points_gen:
		point32 = Point32()
		point32.x = point[0]
		point32.y = point[1]
		point32.z = point[2]
		points_list.append(point32)

	pointCloud = PointCloud()
	pointCloud.header = pointCloud2.header
	pointCloud.points = points_list
	pointCloud.channels = []*len(pointCloud.points)

	return pointCloud


def odometry2PoseStamped(odometry):
	poseStamped = PoseStamped()
	poseStamped.header = odometry.header
	poseStamped.pose = odometry.pose.pose
	return poseStamped


def occupancyGridData2StaticMap(occupancyGrid):
	staticMap = numpy.asarray(occupancyGrid.data, dtype=numpy.int8).reshape(occupancyGrid.info.height, occupancyGrid.info.width)
	staticMap = numpy.transpose(staticMap)
	return staticMap


def mapids2mapCoordination(mapIds, occupancyGrid):
	if mapIds.shape == (2,):        #for 1 id case
		mapIds = numpy.array([mapIds])

	pointsGridCoordinations = mapIds*occupancyGrid.info.resolution
	pointsGridCoordinations = pointsGridCoordinations + [occupancyGrid.info.origin.position.x, occupancyGrid.info.origin.position.y]

	if pointsGridCoordinations.shape == (1,2):     #for 1 id case
		pointsGridCoordinations = pointsGridCoordinations[0]
	return pointsGridCoordinations


def occupancyGridData2PointList(occupancyGrid):
	staticMap = occupancyGridData2StaticMap(occupancyGrid)
	obsIds = numpy.transpose(numpy.nonzero(staticMap))
	pointList = mapids2mapCoordination(obsIds, occupancyGrid)
	return pointList


# basic functions -----
def checkFrameId(stampedData0, stampedData1):
	check = (stampedData0.header.frame_id == stampedData1.header.frame_id)
	if not check:
		rospy.logwarn('frame id missmatch {0}!={1}'.format(stampedData0.header.frame_id, stampedData1.header.frame_id))
	return check


def stampSlector(stampedData0, stampedData1, extrapolation=False):
	if extrapolation:
		if stampedData0.header.stamp > stampedData1.header.stamp:
			stamp = stampedData0.header.stamp
		else:
			stamp = stampedData1.header.stamp
	else:
		if stampedData0.header.stamp < stampedData1.header.stamp:
			stamp = stampedData0.header.stamp
		else:
			stamp = stampedData1.header.stamp
	return stamp


def distPoseStamped2PoseStamped(poseStamped0, poseStamped1, extrapolation=False):
	check = checkFrameId(poseStamped0, poseStamped1)

	dist = distPoint2Point((poseStamped0.pose.position.x, poseStamped0.pose.position.y), (poseStamped1.pose.position.x, poseStamped1.pose.position.y))
	stamp = stampSlector(poseStamped0, poseStamped1, extrapolation)

	return dist, stamp


def distTwist2Twist(twist0, twist1):
	linear_dist = distPoint2Point((twist0.linear.x, twist0.linear.y), (twist1.linear.x, twist1.linear.y))
	return linear_dist


# ROS distance functions -----
def distPoseStamped2Odometry(poseStamped, odometry, extrapolation=False):
	# just thinking 2D (x,y)
	poseStamped_odometry = odometry2PoseStamped(odometry)
	dist, stamp = distPoseStamped2PoseStamped(poseStamped, poseStamped_odometry, extrapolation)
	return dist, stamp


def distOdometry2Odometry(odometry0, odometry1, extrapolation=False):
	poseStamped0 = odometry2PoseStamped(odometry0)
	poseStamped1 = odometry2PoseStamped(odometry1)
	dist, stamp = distPoseStamped2PoseStamped(poseStamped0, poseStamped1, extrapolation)
	return dist, stamp


def distPoseStamped2Path(poseStamped, path, extrapolation=False):
	check = checkFrameId(poseStamped, path)

	# just thinking 2D (x,y)
	pathDists = []
	# TODO just thinking 2D
	pathList = [(iPoseStamped.pose.position.x, iPoseStamped.pose.position.y) for iPoseStamped in path.poses]
	point = (poseStamped.pose.position.x, poseStamped.pose.position.y)
	dist = distPoint2LineString(point, pathList)

	stamp = stampSlector(path, poseStamped, extrapolation)
	return dist, stamp


def distPoseStamped2OccupancyGrid(poseStamped, occupancyGrid, extrapolation=False):
	check = checkFrameId(poseStamped, occupancyGrid)

	point = (poseStamped.pose.position.x, poseStamped.pose.position.y)
	mapPoints = occupancyGridData2PointList(occupancyGrid)
	dist = distMultiPoint2Point(mapPoints, point)

	stamp = stampSlector(poseStamped, occupancyGrid, extrapolation)
	return dist, stamp


def distOdometry2OccupancyGrid(odometry, occupancyGrid, extrapolation=False):
	check = checkFrameId(odometry, occupancyGrid)

	poseStamped = odometry2PoseStamped(odometry)
	dist, stamp = distPoseStamped2OccupancyGrid(poseStamped, occupancyGrid, extrapolation)

	return dist, stamp


def distPath2OccupancyGrid(path, occupancyGrid, extrapolation=False):
	check = checkFrameId(path, occupancyGrid)

	mapPoints = occupancyGridData2PointList(occupancyGrid)
	pathList = [(iPoseStamped.pose.position.x, iPoseStamped.pose.position.y) for iPoseStamped in path.poses]
	dist = distMultiPoint2LineString(mapPoints, pathList)

	stamp = stampSlector(path, occupancyGrid, extrapolation)
	return dist, stamp


def distPointCloud2OccupancyGrid(pointCloud, occupancyGrid, extrapolation=False):
	check = checkFrameId(pointCloud, occupancyGrid)

	mapPoints = occupancyGridData2PointList(occupancyGrid)
	LiderPoints = [ (i.x, i.y) for i in pointCloud.points]
	dist = distMultiPoint2MultiPoint(LiderPoints, mapPoints)

	stamp = stampSlector(pointCloud, occupancyGrid, extrapolation)
	return dist, stamp


def occupancyGridPlot(ax, occupancyGrid):
	staticMap = occupancyGridData2StaticMap(occupancyGrid)
	extent = [occupancyGrid.info.origin.position.x, occupancyGrid.info.width*occupancyGrid.info.resolution  + occupancyGrid.info.origin.position.x,
				occupancyGrid.info.origin.position.y, occupancyGrid.info.height*occupancyGrid.info.resolution + occupancyGrid.info.origin.position.y]
	ax.imshow(staticMap, cmap=plt.cm.gray, origin='lower', extent=extent)
	obsIds = numpy.transpose(numpy.nonzero(staticMap))
	minId = obsIds.min(axis=0)
	maxId = obsIds.max(axis=0)
	minCd = mapids2mapCoordination(minId, occupancyGrid)
	maxCd = mapids2mapCoordination(maxId, occupancyGrid)
	space = 1.0
	left = minCd[0]-space
	right = maxCd[0]+space
	bottom = minCd[1]-space
	top = maxCd[1]+space
	ax.set_xlim([left, right])
	ax.set_ylim([bottom, top])
	ax.set_xlabel('x [m]')
	ax.set_ylabel('y [m]')
	#ax.set_aspect('equal', adjustable='box')
	#plt.tight_layout()
