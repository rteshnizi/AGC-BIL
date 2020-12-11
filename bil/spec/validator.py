import re as RegEx
import json
from abc import ABC, abstractmethod

from bil.model.polygonalRegion import PolygonalRegion
from bil.observation.pose import Pose
from bil.utils.geometry import Geometry

class AbstractValidator(ABC):
	@abstractmethod
	def execute(self, p: Pose):
		"""The method to execute this validator"""
		pass

class RegionValidator(AbstractValidator):
	def __init__(self, funcStr: str) -> bool:
		super().__init__()
		self._str = funcStr
		self._function = eval(funcStr)

	def execute(self, p: Pose):
		pass

class Validator(object):
	def __init__(self, name, funcStr):
		self.name = name
		# for Time, value is a float
		# myStr = "from bil.utils.geometry import Geometry;" + funcStr + ";"
		self._function = eval(funcStr)
		# for Region, value is a polygonalRegion
		if self.isRegion:
			m = RegEx.search(r"(\[[\d,\s\[\]-]*\])", funcStr)
			pts = m.group(0)
			pts = json.loads(pts)
			self.value = PolygonalRegion("sym-%s" % self.name, pts, "BLUE")
		else:
			# TODO: Time executioner
			pass

	@property
	def isRegion(self):
		"""
		FIXME: Just for the sake of debugging, anything beginning with T is a time specifier, other symbols are regions
		"""
		return not self.name.startswith("T")

	def __repr__(self):
		return self._symbolFunc

	def execute(self, p):
		self._function(p)
