import re as RegEx
import json
from abc import ABC, abstractmethod

from bil.model.polygonalRegion import PolygonalRegion
from bil.observation.pose import Pose
from bil.spec.lambdas import Prototypes, LambdaType
import bil

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
		self._funcString = funcStr
		# for Region, value is a polygonalRegion
		if self.isRegion:
			self.value = PolygonalRegion("sym-%s" % self.name, self._function.spaceTimeSet, "BLUE")
		else:
			# TODO: Time executioner
			pass

	@property
	def isRegion(self):
		"""
		Just for the sake of debugging
		"""
		return self._function.type == LambdaType.Region

	def __repr__(self):
		return self._funcString

	def execute(self, p):
		return self._function(p)
