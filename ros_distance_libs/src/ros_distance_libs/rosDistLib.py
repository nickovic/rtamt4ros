import rospy
import numpy
import sensor_msgs.point_cloud2

from std_msgs.msg import String
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2, PointCloud, LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

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


def occupancyGridData2staticMap(occupancyGrid):
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

        dist = distPoint2Point(poseStamped0.pose.position.x, poseStamped0.pose.position.y, poseStamped1.pose.position.x, poseStamped1.pose.position.y)

        stamp = stampSlector(poseStamped0, poseStamped1, extrapolation)

        return dist, stamp


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


def distPoints2pose(points, pose):
        # TODO abolish
        # just thinking 2D (x,y)
        if points.shape == (2,):        #for 1 id case
                points = numpy.array([points])
        dists = points - numpy.array([pose.position.x, pose.position.y])
        dists = numpy.square(dists)
        dists = numpy.sum(dists,axis=1)
        dists = numpy.sqrt(dists)
        if dists.shape == (1,1):     #for 1 id case
                dists = dists[0]
        return dists


def distPoseStamped2pointCloud2(poseStamped, pointCloud2, extrapolation=False):
        check = checkFrameId(poseStamped, pointCloud2)

        points = sensor_msgs.point_cloud2.read_points(pointCloud2)
        # TODO just thinkin 2D
        points_list = numpy.array([(i[0],i[1] )for i in points])
        dists = distPoints2pose(points_list, poseStamped.pose)

        stamp = stampSlector(poseStamped, pointCloud2, extrapolation)
        return dists, stamp


def distPoints2path(points, path):
        # just thinking 2D (x,y)
        # TODO abolish
        # TODO all numpy!
        pathDists = []
        for poseStamped in path.poses:
                dists = distPoints2pose(points, poseStamped.pose)
                dist = numpy.min(dists)
                pathDists.append(dist)
        pathDists = numpy.array(pathDists)
        pathDist = numpy.min(pathDists)
        return pathDist


def distPath2occupancyGrid(path, occupancyGrid, extrapolation=False):
        check = checkFrameId(path, occupancyGrid)

        staticMap = occupancyGridData2staticMap(occupancyGrid)
        obsIds = numpy.transpose(numpy.nonzero(staticMap))
        mapPoints = mapids2mapCoordination(obsIds, occupancyGrid)
        dists = distPoints2path(mapPoints, path)

        stamp = stampSlector(path, occupancyGrid, extrapolation)
        return dists, stamp


def occupancyGridPlot(ax, occupancyGrid):
        staticMap = occupancyGridData2staticMap(occupancyGrid)
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
