# TODO:
# 1) Other agent senario
# 2) monitor with publisher and rtp polot.

import rospy
import numpy
import sensor_msgs.point_cloud2

from std_msgs.msg import String
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2, PointCloud, LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point32

from shapelyLib import *

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


def distsPointCloud2(pointCloud2):
	points_gen = sensor_msgs.point_cloud2.read_points(pointCloud2)
	dists = []
	for point in points_gen:
		point32 = Point32()
		point32.x = point[0]
		point32.y = point[1]
		point32.z = point[2]

		dists.append(point32)


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


def occupancyGridData2StaticMap(occupancyGrid):
	staticMap = numpy.asarray(occupancyGrid.data, dtype=numpy.int8).reshape(occupancyGrid.info.height, occupancyGrid.info.width)
	return staticMap


def mapids2mapCoordination(mapIds, occupancyGrid):
	if mapIds.shape == (2,):        #for 1 id case
		mapIds = numpy.array([mapIds])
	pointsGridCoordinations = mapIds*occupancyGrid.info.resolution
	#TODO condider TF!
	pointsGridCoordinations = pointsGridCoordinations + [occupancyGrid.info.origin.position.x, occupancyGrid.info.origin.position.y]
	if pointsGridCoordinations.shape == (1,2):     #for 1 id case
		pointsGridCoordinations = pointsGridCoordinations[0]
	return pointsGridCoordinations


def distPoseStamped2PoseStamped(poseStamped0, poseStamped1, extrapolation=False):
	check = checkFrameId(poseStamped0, poseStamped1)

	dist = distPoint2Point((poseStamped0.pose.position.x, poseStamped0.pose.position.y), (poseStamped1.pose.position.x, poseStamped1.pose.position.y))

	stamp = stampSlector(poseStamped0, poseStamped1, extrapolation)

	return dist, stamp


def distTwist2Twist(twist0, twist1):
	linear_dist = distPoint2Point((twist0.linear.x, twist0.linear.y), (twist1.linear.x, twist1.linear.y))
	return linear_dist


def odometry2PoseStamped(odometry):
	poseStamped = PoseStamped()
	poseStamped.header = odometry.header
	poseStamped.pose = odometry.pose.pose
	return poseStamped


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


def distPoseStamped2PointCloud2(poseStamped, pointCloud2, extrapolation=False):
	check = checkFrameId(poseStamped, pointCloud2)

	points_gen = sensor_msgs.point_cloud2.read_points(pointCloud2)
	# TODO just thinking 2D
	points_list = numpy.array([(i[0], i[1])for i in points_gen])
	dists = distPoints2Point(points_list, [poseStamped.pose.position.x, poseStamped.pose.position.y])

	stamp = stampSlector(poseStamped, pointCloud2, extrapolation)
	return dists, stamp


def distPoints2Path(points, path):
	# just thinking 2D (x,y)
	pathDists = []
	# TODO just thinking 2D
	pathList = [(poseStamped.pose.position.x, poseStamped.pose.position.y) for poseStamped in path.poses]
	for point in points:
		dist = distPoint2LineString(point, pathList)
		pathDists.append(dist)
	pathDists = numpy.array(pathDists)
	pathDist = numpy.min(pathDists)
	return pathDist


def distPath2OccupancyGrid(path, occupancyGrid, extrapolation=False):
	check = checkFrameId(path, occupancyGrid)

	staticMap = occupancyGridData2StaticMap(occupancyGrid)
	obsIds = numpy.transpose(numpy.nonzero(staticMap))
	mapPoints = mapids2mapCoordination(obsIds, occupancyGrid)
	dists = distPoints2Path(mapPoints, path)

	stamp = stampSlector(path, occupancyGrid, extrapolation)
	return dists, stamp


def distPointCloud2OccupancyGrid(pointCloud, occupancyGrid, extrapolation=False):
	check = checkFrameId(pointCloud, occupancyGrid)

	staticMap = occupancyGridData2StaticMap(occupancyGrid)
	obsIds = numpy.transpose(numpy.nonzero(staticMap))
	mapPoints = mapids2mapCoordination(obsIds, occupancyGrid)
	point_list = [ (i.x, i.y) for i in pointCloud.points]
	dists = distPoints2Points(numpy.array(point_list), numpy.array(mapPoints))

	stamp = stampSlector(pointCloud, occupancyGrid, extrapolation)
	return dists, stamp


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
