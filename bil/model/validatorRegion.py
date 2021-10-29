from shapely.geometry import Polygon
from bil.model.polygonalRegion import PolygonalRegion
from bil.utils.geometry import Geometry

class ValidatorRegion(PolygonalRegion):
	"""
	coords will be used to create the polygon.
	If polygon is given, coords arg will be ignored.
	"""
	def __init__(self, name: str, coords: Geometry.CoordsList, inFov=False):
		super().__init__(name, coords, "RED" if inFov else "BLUE")
		self.inFov = inFov

	def render(self, canvas):
		super().render(canvas, renderText=True)
