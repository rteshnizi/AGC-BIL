from bil.model.polygonalRegion import PolygonalRegion
from bil.utils.geometry import Geometry

COLOR_PALETTE = ["Green", "Purple", "Gold"]
NUM_COLORS = len(COLOR_PALETTE)

class SensingRegion(PolygonalRegion):
	def __init__(self, name, coords, timestamp, idNum, sortCoordinatesClockwise=False):
		super().__init__(name, coords, COLOR_PALETTE[idNum % NUM_COLORS], sortCoordinatesClockwise=sortCoordinatesClockwise)
		self.timestamp = timestamp
		self._renderLineWidth = 4

	def isBeam(self, l):
		return Geometry.lineAndPolygonIntersect(l, self.polygon)

	def render(self, canvas):
		super().render(canvas, False)
