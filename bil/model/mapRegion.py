from bil.model.polygonalRegion import PolygonalRegion

class MapRegion(PolygonalRegion):
	def __init__(self, name, coords, typeStr, featureDefinition):
		self.isObstacle: bool = featureDefinition.easyToCross.car < 100
		super().__init__(name, coords, "Grey", "Black" if self.isObstacle else "")
		self.type = typeStr

	def render(self, canvas, renderText=False):
		super().render(canvas, renderText)
