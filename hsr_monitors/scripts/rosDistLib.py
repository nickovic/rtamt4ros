import numpy

from xythLib import *

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


def distPoints2pose(points, pose):
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
