import numpy
from numpy.linalg import norm
from shapely.geometry import Polygon, LineString, Point, MultiPoint
from scipy import signal, interpolate


class xythCoord:
    x = []
    y = []
    th = []
    time = []


    def __init__(self, x, y, th, time):
        self.x = x
        self.y = y
        self.th = th
        self.time = time


    def __sub__(self, other):
        subX = self.x - other.x
        subY = self.y - other.y
        subTh = self.th - other.th
        subTime = self.time - other.time
        return xythCoord(subX, subY, subTh, subTime)


    def __add__(self, other):
        subX = self.x + other.x
        subY = self.y + other.y
        subTh = self.th + other.th
        subTime = self.time + other.time
        return xythCoord(subX, subY, subTh, subTime)


def posSizeRect2Poly(rectPos, rectSize):
    rectSizeH = numpy.array(rectSize)/2.0
    polyRect = Polygon([(rectPos[0]+rectSizeH[0], rectPos[1]+rectSizeH[1]),
                        (rectPos[0]-rectSizeH[0], rectPos[1]+rectSizeH[1]),
                        (rectPos[0]-rectSizeH[0], rectPos[1]-rectSizeH[1]),
                        (rectPos[0]+rectSizeH[0], rectPos[1]-rectSizeH[1])])
    return polyRect


def distMultiPoint2Point(multiPoint, point):
    if not type(multiPoint) is MultiPoint:
        multiPoint = MultiPoint(multiPoint)
    if not type(point) is Point:
        point = Point(point)
    dist = point.distance(multiPoint)
    return dist


def distMultiPoint2MultiPoint(multiPoint0, multiPoint1):
    # just thinking 2D (x,y)
    if not type(multiPoint0) is MultiPoint:
        multiPoint0 = MultiPoint(multiPoint0)
    if not type(multiPoint1) is MultiPoint:
        multiPoint1 = MultiPoint(multiPoint1)
    dist = multiPoint0.distance(multiPoint1)
    return dist


def distPoint2Point(point1, point2):
    if not type(point1) is Point:
        point1 = Point(point1)
    if not type(point2) is Point:
        point2 = Point(point2)
    dist = point1.distance(point2)
    return dist


def distPoly2Point(poly, point):
    #defining sigined distance
    dist = poly.exterior.distance(point)
    if (point.within(poly)):
        dist = -dist
    return dist


def distP2R(rectPos, rectSize, pointPos):
    #TODO consider rotation
    #TODO abolish
    #Pos and Size are 2D
    polyRect = posSizeRect2Poly(rectPos, rectSize)
    point = Point(pointPos[0], pointPos[1])
    dist = distPoly2Point(polyRect, point)
    return dist


def distTraj2Poly(poly, trajX, trajY):
    distTraj = []
    for ix, iy in zip(trajX, trajY):
        point = Point(ix[1], iy[1])
        dist = distPoly2Point(poly, point)
        distTraj.append( (ix[0], dist) )
    return distTraj


def distTraj2Rect(rectPos, rectSize, trajX, trajY):
    polyRect = posSizeRect2Poly(rectPos, rectSize)
    distTraj = distTraj2Po(polyRect, trajX, trajY)
    return distTraj


# TODO: Finihs this
def distLineStr2Po(poly, line):
    #defining sigined distance
    dist = poly.exterior.distance(line)
#    if(line.crosses(poly)):
#        dist = poly.exterior.distance(line.interpolate(0.0001))
    return dist


# TODO: Finihs this
def distLineStr2R(rectPos, rectSize, line):
    polyRect = posSizeRect2Poly(rectPos, rectSize)
    dist = polyRect.exterior.distance(line)
    return dist


def distPoint2LineString(point, lineString):
    if not type(point) is Point:
        point = Point(point)
    if not type(lineString) is LineString:
        lineString = LineString(lineString)
    dist = lineString.distance(point)
    return dist


def distMultiPoint2LineString(multiPoint, lineString):
    if not type(multiPoint) is MultiPoint:
        multiPoint = MultiPoint(multiPoint)
    if not type(lineString) is LineString:
        lineString = LineString(lineString)
    dist = lineString.distance(multiPoint)
    return dist


def distLineString2LineString(lineString0, lineString1):
    if not type(lineString0) is LineString:
        lineString0 = LineString(lineString0)
    if not type(lineString1) is LineString:
        lineString1 = LineString(lineString1)
    dist = lineString0.distance(lineString1)
    return dist


def closetLineOfWaypoits(tPoint, wayPoints):
    lines = []
    dists = []
    for i in range(1, len(wayPoints.coords)):
        sPoint = list(wayPoints.coords)[i-1]
        ePoint = list(wayPoints.coords)[i]
        line = LineString([sPoint, ePoint])
        #print(list(line.coords))
        lines.append(line)

        dist = line.distance(tPoint)
        #print(str(dist))
        dists.append(dist)

    mi = numpy.argmin(numpy.array(dists))
    cDist = dists[mi]
    cLine = lines[mi]
    #print(mi)
    #print(str(list(cLine.coords)) + ' ' + str(cDist))

    return cLine, cDist


def signedDistansL2P(line, point):
    sPoint = numpy.array(list(line.coords)[0])
    ePoint = numpy.array(list(line.coords)[1])
    oPoint = numpy.array(point.coords)
    sDist = numpy.cross(sPoint-ePoint, sPoint-oPoint)/norm(sPoint-ePoint)

    return sDist[0]
