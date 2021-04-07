from shapely.geometry import Polygon
from bil.model.polygonalRegion import PolygonalRegion
from bil.utils.geometry import Geometry

COLOR_PALETTE = ["Green", "Purple", "Gold"]
NUM_COLORS = len(COLOR_PALETTE)

class SensingRegion(PolygonalRegion):
	"""
	coords will be used to create the polygon.
	If polygon is given, coords arg will be ignored.
	"""
	def __init__(self, name: str, coords: Geometry.CoordsList, timestamp: float, idNum: int, polygon: Polygon = None):
		super().__init__(name, coords, COLOR_PALETTE[idNum % NUM_COLORS], polygon=polygon)
		self.timestamp = timestamp
		self._renderLineWidth = 4

	def isBeam(self, l):
		return Geometry.lineAndPolygonIntersect(l, self.polygon)

	def render(self, canvas):
		super().render(canvas, False)
