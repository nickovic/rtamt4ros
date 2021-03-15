import rospy
import numpy

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from xythLib import *

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


def distTwist2Twist(twist0, twist1):
        # TODO abolish
        linear_dist = distP2P(twist0.linear.x, twist0.linear.y, twist1.linear.x, twist1.linear.y)
        return linear_dist


def distPoseStamped2PoseStamped(poseStamped0, poseStamped1, extrapolation=False):
        if poseStamped0.header.frame_id != poseStamped1.header.frame_id:
                rospy.logwarn("frame id missmatch {0}!={1}".format(poseStamped0.header.frame_id, poseStamped1.header.frame_id))

        dist = distP2P(poseStamped0.pose.position.x, poseStamped0.pose.position.y, poseStamped1.pose.position.x, poseStamped1.pose.position.y)
        if extrapolation:
                time = max(poseStamped0.header.stamp.to_sec(), poseStamped1.header.stamp.to_sec())
        else:
                time = min(poseStamped0.header.stamp.to_sec(), poseStamped1.header.stamp.to_sec())
        return dist, time


def odometry2PoseStamped(odometry):
        poseStamped = PoseStamped()
        poseStamped.header = odometry.header
        poseStamped.pose = odometry.pose.pose
        return poseStamped


def distPoseStamped2Odometry(poseStamped, odometry, extrapolation=False):
        # just thinking 2D (x,y)
        poseStamped_odometry = odometry2PoseStamped(odometry)
        dist, time = distPoseStamped2PoseStamped(poseStamped, poseStamped_odometry, extrapolation)
        return dist, time


def distOdometry2Odometry(odometry0, odometry1, extrapolation=False):
        poseStamped0 = odometry2PoseStamped(odometry0)
        poseStamped1 = odometry2PoseStamped(odometry1)
        dist, time = distPoseStamped2PoseStamped(poseStamped0, poseStamped1, extrapolation)
        return dist, time


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


def distPoints2poses(points, poses):
        # just thinking 2D (x,y)
        # TODO abolish
        # TODO all numpy!
        pathDists = []
        for pose in poses:
                dists = distPoints2pose(points, pose.pose)
                dist = numpy.min(dists)
                pathDists.append(dist)
        pathDists = numpy.array(pathDists)
        pathDist = numpy.min(pathDists)
        return pathDist


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
