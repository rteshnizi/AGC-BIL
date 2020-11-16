from bil.model.polygonalRegion import PolygonalRegion

class SpecSymbol(object):
	def __init__(self, name, symbolDict):
		self.REGION_TYPE = "region"
		self._symbolDict = symbolDict
		self.name = name
		self.type = symbolDict["type"]
		# for Time, value is a float
		self.value = symbolDict["value"]
		# for Region, value is a polygonalRegion
		if self.type == self.REGION_TYPE:
			self.value = PolygonalRegion("sym-%s" % self.name, self.value, "BLUE")

	@property
	def isRegion(self):
		return self.type == self.REGION_TYPE
