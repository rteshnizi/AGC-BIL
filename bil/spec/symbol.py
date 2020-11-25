from bil.model.polygonalRegion import PolygonalRegion
import re as RegEx
import json

class SpecAction(object):
	def __init__(self, name, symbolFunc):
		self.name = name
		# for Time, value is a float
		self.function = symbolFunc
		# for Region, value is a polygonalRegion
		if self.isRegion:
			m = RegEx.search(r"(\[[\d,\s\[\]-]*\])", self.function)
			pts = m.group(0)
			pts = json.loads(pts)
			self.value = PolygonalRegion("sym-%s" % self.name, pts, "BLUE")
		else:
			pass

	@property
	def isRegion(self):
		"""
		FIXME: Just for the sake of debugging, anything beginning with T is a time specifier, other symbols are regions
		"""
		return not self.name.startswith("T")
