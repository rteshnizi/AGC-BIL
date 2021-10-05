from shapely.geometry import LineString, MultiLineString, Point, Polygon, MultiPolygon
from shapely.ops import unary_union
from typing import List, Union, Tuple, Dict, Callable
from math import pi as PI
from math import sqrt, cos, sin, atan2, degrees, inf
from functools import reduce
from skimage import transform
from scipy.spatial.transform import Rotation, Slerp
import operator
import numpy as np

class Geometry:
	EPSILON = 0.001
	Vector = Tuple[float, float]
	Coords = Tuple[float, float]
	CoordsList = List[Coords]
	CoordsMap = Dict[Coords, Coords]

	@staticmethod
	def vectorsAreEqual(vect1: Vector, vect2: Vector, withinEpsilon = True) -> bool:
		d1 = Geometry.distanceFromVects(vect1, vect2)
		margin = Geometry.EPSILON if withinEpsilon else 0
		return d1 <= margin

	@staticmethod
	def coordsAreAlmostEqual(coords1: Coords, coords2: Coords) -> bool:
		d1 = Geometry.distance(coords1[0], coords1[1], coords2[0], coords2[1])
		return d1 <= 0.1 # EPSILON is too large unfortunately

	@staticmethod
	def lineSegmentsAreAlmostEqual(l1: LineString, l2: LineString) -> bool:
		l1Coords = list(l1.coords)
		l2Coords = list(l2.coords)
		if Geometry.coordsAreAlmostEqual(l1Coords[0], l2Coords[0]) and Geometry.coordsAreAlmostEqual(l1Coords[1], l2Coords[1]): return True
		if Geometry.coordsAreAlmostEqual(l1Coords[0], l2Coords[1]) and Geometry.coordsAreAlmostEqual(l1Coords[1], l2Coords[0]): return True
		return False

	@staticmethod
	def vectLength(x, y) -> float:
		return sqrt((y * y) + (x * x))

	@staticmethod
	def sortCoordinatesClockwise(coords: list):
		"""
		(x,y)

		coords = [(0, 1), (1, 0), (1, 1), (0, 0)]
		"""
		center = tuple(map(operator.truediv, reduce(lambda x, y: map(operator.add, x, y), coords), [len(coords)] * 2))
		sortedCoords = sorted(coords, key=lambda coord: (-135 - degrees(atan2(*tuple(map(operator.sub, coord, center))[::-1]))) % 360)
		return sortedCoords

	@staticmethod
	def isLineSegment(l: LineString) -> bool:
		return isinstance(l, LineString) and len(l.coords) == 2

	@staticmethod
	def orthogonal(p1, p2) -> float:
		return  (p2[1] - p1[1], -1 * (p2[0] - p1[0]))

	@staticmethod
	def midpoint(p1, p2) -> float:
		return Geometry.midpointXy(p1[0], p1[1], p2[0], p2[1])

	@staticmethod
	def midpointXy(x1, y1, x2, y2) -> float:
		return ((x1 + x2) / 2, (y1 + y2) / 2)

	@staticmethod
	def slope(x1, y1, x2, y2) -> float:
		return (y2 - y1) / (x2 - x1)

	@staticmethod
	def distance(x1, y1, x2, y2) -> float:
		vect = Geometry.distanceVect(x1, y1, x2, y2)
		length = Geometry.vectLength(vect[0], vect[1])
		return length

	@staticmethod
	def distanceFromVects(vect1: Vector, vect2: Vector) -> float:
		return Geometry.distance(vect1[0], vect1[1], vect2[0], vect2[1])

	@staticmethod
	def distanceVect(x1, y1, x2, y2) -> Vector:
		dx = x2 - x1
		dy = y2 - y1
		return (dx, dy)

	@staticmethod
	def distanceVectFromPts(pt1: Point, pt2: Point) -> Vector:
		return Geometry.distanceVect(pt1.x, pt1.y, pt2.x, pt2.y)

	@staticmethod
	def getUnitVector(x, y) -> Vector:
		"""
		Returns a tuple (x, y) of a vector.
		"""
		length = Geometry.distance(0, 0, x, y)
		return (x / length, y / length)

	@staticmethod
	def getUnitVectorFromAngle(theta) -> Vector:
		"""
		theta is the angle w.r.t X axis
		Returns a tuple (x, y) of a vector.
		"""
		x = cos(theta)
		y = sin(theta)
		return Geometry.getUnitVector(x, y)

	@staticmethod
	def pushPointEpsilon(x: float, y: float, pt: Point) -> Point:
		"""
		Push point epsilon in the direction of the given x, y.
		"""
		(x, y) = Geometry.getUnitVector(x, y)
		y = y * Geometry.EPSILON
		x = x * Geometry.EPSILON
		return Point(pt.x + x, pt.y + y)

	@staticmethod
	def getDirectionXyFromLineSeg(l: LineString) -> tuple:
		"""
		Returns (x, y) tuple
		"""
		if not Geometry.isLineSegment(l):
			raise RuntimeError("This method is only tested for line segments")
		coords = list(l.coords)
		x = coords[1][0] - coords[0][0]
		y = coords[1][1] - coords[0][1]
		return (x, y)

	@staticmethod
	def isPointOnLine(p: Point, l: LineString) -> bool:
		return l.distance(p) <= Geometry.EPSILON

	@staticmethod
	def isXyInsidePolygon(ptX: float, ptY: float, polygon: Polygon) -> bool:
		return Geometry.isPointInsidePolygon(Point(ptX, ptY), polygon)

	@staticmethod
	def isPointInsidePolygon(pt: Point, polygon: Polygon) -> bool:
		return polygon.contains(pt)

	@staticmethod
	def pointStringId(x: float, y: float) -> str:
		return "%.2f,%.2f" % (x, y)

	@staticmethod
	def lineSegStringId(line: LineString) -> frozenset:
		return Geometry.coordListStringId(list(line.coords))

	@staticmethod
	def coordListStringId(coords: CoordsList) -> frozenset:
		return frozenset([Geometry.pointStringId(coord[0], coord[1]) for coord in coords])

	@staticmethod
	def lineAndPolygonIntersect(l: LineString, p: Polygon) -> bool:
		# TODO: A line seg might touch but not intersect
		return l.intersects(p)

	@staticmethod
	def lineAndPolygonIntersectFromXy(xStart, yStart, xEnd, yEnd, p: Polygon) -> bool:
		# TODO: A line seg might touch but not intersect
		l = LineString([(xStart, yStart), (xEnd, yEnd)])
		return Geometry.lineAndPolygonIntersect(l, p)

	@staticmethod
	def createLine(xStart, yStart, xEnd, yEnd) -> LineString:
		# TODO: A line seg might touch but not intersect
		return LineString([(xStart, yStart), (xEnd, yEnd)])

	@staticmethod
	def lineSegmentSlope(segment: LineString) -> float:
		x11, y11 = segment.coords[0]
		x12, y12 = segment.coords[1]
		if x12 - x11 == 0: return inf
		slope = (y12 - y11) / (x12 - x11)
		return slope

	@staticmethod
	def polygonAndPolygonIntersect(p1: Polygon, p2: Polygon) -> bool:
		return p1.intersects(p2)

	@staticmethod
	def union(polyList: List[Polygon]) -> Polygon:
		return unary_union(polyList)

	@staticmethod
	def subtract(poly1: Polygon, poly2: Polygon) -> Polygon:
		return poly1.difference(poly2)

	@staticmethod
	def intersectLineSegments(l1: LineString, l2: LineString):
		if not (Geometry.isLineSegment(l1) and Geometry.isLineSegment(l2)):
			raise RuntimeError("This method is only tested for line segments")
		r = l1.intersection(l2)
		if isinstance(r, LineString) and r.is_empty:
			return None
		return r

	@staticmethod
	def intersect(p1: Polygon, p2: Polygon) -> Union[List[Polygon], MultiPolygon]:
		"""
		Returns a List of the instersection polygon(s)
		"""
		intersection = p1.intersection(p2)
		try:
			if len(intersection) > 0:
				return intersection
			raise RuntimeError("intersection should never be empty")
		except:
			# Assumption here is that if it throws it's a Polygon
			return [intersection]

	@staticmethod
	def shadows(mapRegionPoly: Polygon, fovPoly: Polygon) -> Union[List[Polygon], MultiPolygon]:
		nonoverlap = mapRegionPoly.difference(fovPoly) # difference() is wrong because it excludes the boundaries of fov from the shadows
		# nonoverlap = mapRegionPoly.difference(mapRegionPoly.intersection(fovPoly)) # difference() is wrong because it excludes the boundaries of fov from the shadows
		# nonoverlap = mapRegionPoly.symmetric_difference(fovPoly).difference(fovPoly)
		try:
			if len(nonoverlap) > 0:
				return nonoverlap
			raise RuntimeError("Nonoverlap should never be empty")
		except:
			# Assumption here is that if it throws there is one element
			return [nonoverlap]

	@staticmethod
	def _haveOverlappingEdge(p1: Polygon, p2: Polygon) -> bool:
		"""
		DEPRECATED: https://github.com/Toblerity/Shapely/issues/1101
		"""
		# if p1.touches(p2):
		r = p1.intersection(p2)
		if isinstance(r, LineString) or isinstance(r, MultiLineString):
			return True if r.length > 0 else False
		return False

	@staticmethod
	def haveOverlappingEdge(p1: Polygon, p2: Polygon) -> bool:
		"""
		Previous implementation used intersection operator. But it turn out to be VERY buggy (see _haveOverlappingEdge)
		This one will iterate over all of the boundary edges check if lines are parallel and sees if the distance is minimal.
		"""
		shapelyIntersectionCheck = Geometry._haveOverlappingEdge(p1, p2)
		if p1.distance(p2) > 0: return False # This is an important optimization. The process below is time consuming
		if shapelyIntersectionCheck: return True
		lineSegments1 = list(map(LineString, zip(p1.exterior.coords[:-1], p1.exterior.coords[1:])))
		lineSegments2 = list(map(LineString, zip(p2.exterior.coords[:-1], p2.exterior.coords[1:])))
		for lineSeg1 in lineSegments1:
			slope1 = Geometry.lineSegmentSlope(lineSeg1)
			for lineSeg2 in lineSegments2:
				slope2 = Geometry.lineSegmentSlope(lineSeg2)
				if abs(slope1 - slope2) > Geometry.EPSILON: continue
				d = lineSeg1.distance(lineSeg2)
				if d < Geometry.EPSILON:
					return True
		return False

	@staticmethod
	def getAllIntersectingEdges(line: LineString, polygon: Union[Polygon, MultiPolygon]) -> List[LineString]:
		if isinstance(polygon, MultiPolygon):
			raise "I haven't checked the API to see how to work with this yet."
		edges = []
		verts = list(polygon.exterior.coords)
		polygonBoundary = LineString(verts)
		points = polygonBoundary.intersection(line)
		if points.is_empty: return edges
		points = [points] if isinstance(points, Point) else list(points)
		for point in points:
			for v1, v2 in zip(verts, verts[1:]):
				edge = LineString([v1, v2])
				if Geometry.isPointOnLine(point, edge):
					edges.append(edge)
					break
		return edges

	@staticmethod
	def getAffineTransformation(p: Polygon, pPrime: Polygon, centerOfRotation: Coords) -> transform.AffineTransform:
		"""
		see: https://stackoverflow.com/a/47102206/750567
		"""
		pCoords = np.array(p.exterior.coords)
		pPrimeCoords = np.array(pPrime.exterior.coords)
		pCoords = [(coords[0] - centerOfRotation[0], coords[1] - centerOfRotation[1]) for coords in pCoords]
		pPrimeCoords = [(coords[0] - centerOfRotation[0], coords[1] - centerOfRotation[1]) for coords in pPrimeCoords]
		pCoords = np.array(pCoords)
		pPrimeCoords = np.array(pPrimeCoords)
		# matrix = transform.estimate_transform("affine", pCoords, pPrimeCoords)
		matrix = transform.estimate_transform("similarity", pCoords, pPrimeCoords)
		return matrix

	@staticmethod
	def getParameterizedAffineTransformation(transformation: transform.AffineTransform, param: float) -> transform.AffineTransform:
		"""

			Given an affine transformation and a parameter between 0 and 1, this method returns a linear interpolation transformation.

			### Remarks
			This method returns an affine transformation that provides a linear interpolation of the given transformation, on the following assumptions:
			* `param` is in [0, 1] interval,
			* The affine transformation at `param == 0` is Identity Matrix,
			* The affine transformation at `param == 1` is the given transformation,
			* A slerp method is used to obtain the rotation interpolation.
		"""
		if param > 1 or param < 0:
			raise("Parameter should be in range [0, 1]")
		# Easy cases that do not need calculation
		if param == 0: return transform.AffineTransform(np.identity(3))
		if param == 1: return transform.AffineTransform(transformation.params)
		# Other params
		# scale = [((transformation.scale[0] - 1) * param) + 1, ((transformation.scale[1] - 1) * param) + 1]
		scale = 1
		rotations = Rotation.from_matrix([
			[
				[1, 0, 0],
				[0, 1, 0],
				[0, 0, 1]
			],
			[
				[transformation.params[0][0], transformation.params[0][1], 0],
				[transformation.params[1][0], transformation.params[1][1], 0],
				[0, 0, 1]
			]
		])
		slerp = Slerp([0, 1], rotations)
		rotation = slerp([0, param, 1])[1].as_euler('xyz')[2]
		# shear = transformation.shear * param
		shear = 0
		translation = [transformation.translation[0] * param, transformation.translation[1] * param]
		# translation = [0, 0]
		parameterizedMatrix = transform.AffineTransform(matrix=None, scale=scale, rotation=rotation, shear=shear, translation=translation)
		return parameterizedMatrix

	@staticmethod
	def applyMatrixTransformToPolygon(transformation: transform.AffineTransform, polygon: Polygon, centerOfRotation: Coords) -> Polygon:
		pCoords = list(polygon.exterior.coords)
		pCoords = [(coords[0] - centerOfRotation[0], coords[1] - centerOfRotation[1]) for coords in pCoords]
		pCoords = np.array(pCoords)
		transformedCoords = transform.matrix_transform(pCoords, transformation.params)
		transformedCoords = [(coords[0] + centerOfRotation[0], coords[1] + centerOfRotation[1]) for coords in transformedCoords]
		transformedPolygon = Polygon(transformedCoords)
		return transformedPolygon

	@staticmethod
	def findTheLastTimeTheyAreColliding(movingEdge: LineString, staticEdge: LineString, transformation: transform.AffineTransform) -> float:
		"""
			### Remarks
			This assumes the edges are in contact already.
		"""
		NUM_SAMPLES = 100
		latestTime = inf
		# If the edge is not intersecting currently, we don't need to check for the latest time
		if Geometry.intersectLineSegments(movingEdge, staticEdge) is None:
			return latestTime
		for x in range(1, NUM_SAMPLES, 1):
			fraction = x / NUM_SAMPLES
			newTransform = Geometry.getParameterizedAffineTransformation(transformation, fraction)
			intermediateLine = Geometry.applyMatrixTransformToLineString(newTransform, movingEdge)
			if Geometry.intersectLineSegments(intermediateLine, staticEdge) is not None:
				latestTime = fraction
			else:
				break
		return latestTime

	@staticmethod
	def findTheEarliestTimeTheyAreColliding(movingEdge: LineString, staticEdge: LineString, transformation: transform.AffineTransform) -> float:
		NUM_SAMPLES = 100
		latestTime = inf
		for x in range(0, NUM_SAMPLES, 1):
			fraction = x / NUM_SAMPLES
			newTransform = Geometry.getParameterizedAffineTransformation(transformation, fraction)
			intermediateLine = Geometry.applyMatrixTransformToLineString(newTransform, movingEdge)
			if Geometry.intersectLineSegments(intermediateLine, staticEdge):
				latestTime = fraction
				break
		return latestTime

	@staticmethod
	def applyMatrixTransformToLineString(transformation: transform.AffineTransform, line: LineString, centerOfRotation: Coords) -> LineString:
		pCoords = list(line.coords)
		pCoords = [(coords[0] - centerOfRotation[0], coords[1] - centerOfRotation[1]) for coords in pCoords]
		transformedCoords = transform.matrix_transform(pCoords, transformation.params)
		transformedCoords = [(coords[0] + centerOfRotation[0], coords[1] + centerOfRotation[1]) for coords in transformedCoords]
		transformedLineString = LineString(transformedCoords)
		return transformedLineString

LineString.__repr__ = lambda l: "LS"
Point.__repr__ = lambda p: "P%s" % repr((p.x, p.y))
