from bil.model.polygonalRegion import PolygonalRegion

class ShadowRegion(PolygonalRegion):
	def __init__(self, name, coords):
		super().__init__(name, coords, "BLACK", "GRAY")

	def render(self, canvas, renderText=False):
		super().render(canvas, renderText, hashFill=True, hashDensity=50)
