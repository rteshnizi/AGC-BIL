from shapely.geometry import LineString, Point, Polygon
from typing import Dict, List, Tuple

from bil.gui.drawing import Drawing
from bil.utils.geometry import Geometry

class PolygonalRegion:
	"""
	coords will be used to create the polygon.
	If polygon is given, coords arg will be ignored.
	"""
	def __init__(self, name: str, coords: List[Tuple[float, float]], boundaryColor="RED", backgroundColor="", polygon: Polygon = None):
		self.name = name
		self._renderLineWidth = 1
		try:
			self._coordsList = coords if polygon is None else list(polygon.exterior.coords)
		except:
			print("woopsie")
		self._coordsDict = self._buildCoords(self._coordsList)
		self.polygon = Polygon(self._coordsList) if polygon is None else polygon
		self.BOUNDARY_COLOR = boundaryColor
		self.BACKGROUND_COLOR = backgroundColor
		self._edges = self._buildEdges(self._coordsList)
		self.canvasId = None
		self.textId = None

	def __repr__(self):
		return self.name

	def _buildCoords(self, coords: Geometry.CoordsList) -> Dict[str, Point]:
		d = {}
		for c in coords:
			d[Geometry.pointStringId(c[0], c[1])] = Point(c[0], c[1])
		return d

	def _buildEdges(self, coords):
		d = {}
		for i in range(len(coords) - 1):
			c1 = coords[i]
			c2 = coords[i + 1]
			edgeCoords = [(c1[0], c1[1]), (c2[0], c2[1])]
			d[Geometry.coordListStringId(edgeCoords)] = LineString(edgeCoords)
		c1 = coords[len(coords) - 1]
		c2 = coords[0]
		edgeCoords = [(c1[0], c1[1]), (c2[0], c2[1])]
		d[Geometry.coordListStringId(edgeCoords)] = LineString(edgeCoords)
		return d

	def _hasEdgeByXy(self, x1, y1, x2, y2):
		return Geometry.coordListStringId(x1, y1, x2, y2) in self._edges

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
		if self.canvasId is not None:
			Drawing.RemoveShape(canvas, self.canvasId)
			self.canvasId = None
		if self.textId is not None:
			Drawing.RemoveShape(canvas, self.textId)
			self.textId = None
