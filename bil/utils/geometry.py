from shapely.geometry import LineString, Point, Polygon
from shapely.ops import cascaded_union
from typing import List
from math import sqrt, cos, sin

class Geometry:
	EPSILON = 0.01

	@staticmethod
	def isLineSegment(l: LineString) -> bool:
		return isinstance(l, LineString) and len(l.coords) == 2

	@staticmethod
	def distance(x1, y1, x2, y2) -> float:
		dx = x2 - x1
		dy = y2 - y1
		length = sqrt((dy * dy) + (dx * dx))
		return length

	@staticmethod
	def getUnitVector(x, y) -> tuple:
		"""
		Returns a tuple (x, y) of a vector.
		"""
		length = Geometry.distance(0, 0, x, y)
		return (x / length, y / length)

	@staticmethod
	def getUnitVectorFromAngle(theta) -> tuple:
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
	def isXyInsidePolygon(ptX: float, ptY: float, polygon: Polygon) -> bool:
		return Geometry.isPointInsidePolygon(Point(ptX, ptY), polygon)

	@staticmethod
	def isPointInsidePolygon(pt: Point, polygon: Polygon) -> bool:
		return polygon.contains(pt)

	@staticmethod
	def pointStringId(x: float, y: float) -> str:
		return "%.5f,%.5f" % (x, y)

	@staticmethod
	def lineSegStringId(x1: float, y1: float, x2: float, y2: float) -> frozenset:
		return frozenset([Geometry.pointStringId(x1, y1), Geometry.pointStringId(x2, y2)])

	@staticmethod
	def lineAndPolygonIntersect(l: LineString, p: Polygon):
		# TODO: A line seg might touch but not intersect
		return l.intersects(p)

	@staticmethod
	def lineAndPolygonIntersectFromXy(xStart, yStart, xEnd, yEnd, p: Polygon):
		# TODO: A line seg might touch but not intersect
		l = LineString([(xStart, yStart), (xEnd, yEnd)])
		return Geometry.lineAndPolygonIntersect(l, p)

	@staticmethod
	def createLine(xStart, yStart, xEnd, yEnd) -> LineString:
		# TODO: A line seg might touch but not intersect
		return LineString([(xStart, yStart), (xEnd, yEnd)])

	@staticmethod
	def polygonAndPolygonIntersect(p1: Polygon, p2: Polygon):
		return p1.intersects(p2)

	@staticmethod
	def union(polyList: List[Polygon]) -> Polygon:
		return cascaded_union(polyList)

	@staticmethod
	def intersectLineSegments(l1: LineString, l2: LineString):
		if not (Geometry.isLineSegment(l1) and Geometry.isLineSegment(l2)):
			raise RuntimeError("This method is only tested for line segments")
		r = l1.intersection(l2)
		if isinstance(r, LineString) and r.length == 0:
			return None
		return r

	@staticmethod
	def intersect(p1: Polygon, p2: Polygon) -> List[Polygon]:
		intersection = p1.intersection(p2)
		try:
			if len(intersection) > 0:
				return list(intersection)
			raise RuntimeError("intersection should never be empty")
		except:
			# Assumption here is that if it throws there is one element
			return [intersection]

	@staticmethod
	def nonOverlapping(p1: Polygon, p2: Polygon) -> List[Polygon]:
		nonoverlap = (p1.symmetric_difference(p2)).difference(p2)
		try:
			if len(nonoverlap) > 0:
				return list(nonoverlap)
			raise RuntimeError("Nonoverlap should never be empty")
		except:
			# Assumption here is that if it throws there is one element
			return [nonoverlap]

	@staticmethod
	def commonEdge(p1: Polygon, p2: Polygon):
		if p1.touches(p2):
			r = p1.boundary.intersection(p2.boundary)
			return r if r.length > 0 else None
		return None
