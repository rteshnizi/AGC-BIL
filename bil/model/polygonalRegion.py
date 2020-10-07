from shapely.geometry import LineString, Point, Polygon
from typing import List

from bil.gui.drawing import Drawing
from bil.utils.geometry import Geometry

class PolygonalRegion:
	def __init__(self, name, coords, boundaryColor="RED", backgroundColor=""):
		self.name = name
		self._renderLineWidth = 1
		self._coordsDict = self._buildCoords(coords)
		self._coordsList = coords
		self.polygon = Polygon(coords)
		self.BOUNDARY_COLOR = boundaryColor
		self.BACKGROUND_COLOR = backgroundColor
		self._edges = self._buildEdges(coords)
		self.canvasId = None
		self.textId = None

	def __repr__(self):
		return self.name

	def _buildCoords(self, coords):
		d = {}
		for c in coords:
			d[Geometry.pointStringId(c[0], c[1])] = Point(c[0], c[1])
		return d

	def _buildEdges(self, coords):
		d = {}
		for i in range(len(coords) - 1):
			c1 = coords[i]
			c2 = coords[i + 1]
			d[Geometry.lineSegStringId(c1[0], c1[1], c2[0], c2[1])] = LineString([(c1[0], c1[1]), (c2[0], c2[1])])
		c1 = coords[len(coords) - 1]
		c2 = coords[0]
		d[Geometry.lineSegStringId(c1[0], c1[1], c2[0], c2[1])] = LineString([(c1[0], c1[1]), (c2[0], c2[1])])
		return d

	def _hasEdgeByXy(self, x1, y1, x2, y2):
		return Geometry.lineSegStringId(x1, y1, x2, y2) in self._edges

	def _hasEdgeByName(self, name):
		return name in self._edges

	def isInsideRegion(self, x, y):
		return Geometry.isXyInsidePolygon(x, y, self.polygon)

	def intersectsRegion(self, other: "PolygonalRegion"):
		return Geometry.polygonAndPolygonIntersect(self.polygon, other.polygon)

	def union(self, others: List["PolygonalRegion"]) -> Polygon:
		return Geometry.union([r.polygon for r in others].append(self.polygon))

	def getCommonEdge(self, other) -> LineString:
		for e in self._edges:
			if other._hasEdgeByName(e): return self._edges[e]
		return None

	def render(self, canvas, renderText=False, hashFill=False, hashDensity=25):
		if self.canvasId is not None: self.clearRender(canvas)
		if hashDensity not in [75, 50, 25, 12]: raise AssertionError("Density should be one of 75, 50, 25, or 12.")
		self.canvasId = Drawing.CreatePolygon(canvas, self._coordsList, outline=self.BOUNDARY_COLOR, fill=self.BACKGROUND_COLOR, width=self._renderLineWidth, tag=self.name, hashFill=hashFill, hashDensity=hashDensity)
		if renderText: self.textId = Drawing.CreateText(canvas, [self.polygon.centroid.x, self.polygon.centroid.y], self.name, tag=self.name, color="White" if self.BACKGROUND_COLOR.upper() == "BLACK" else "Black")

	def clearRender(self, canvas):
		if self.canvasId is not None: Drawing.RemoveShape(canvas, self.canvasId)
		if self.textId is not None: Drawing.RemoveShape(canvas, self.textId)
