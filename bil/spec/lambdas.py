from enum import Enum
from functools import partial
from shapely.geometry import Polygon

from bil.spec.spaceTime import SpaceTimeSet
from bil.utils.geometry import Geometry
from bil.utils.python import ObjectLiteral

class LambdaType(Enum):
	Region = 1
	Time = 2

class Lambda:
	def __init__(self, func, spaceTimeDescriptor, lambdaType: LambdaType):
		self._originalFuncName = func.__name__
		if lambdaType == LambdaType.Region:
			self.spaceTimeSet = SpaceTimeSet(Polygon(spaceTimeDescriptor), None)
		else:
			self.spaceTimeSet = SpaceTimeSet(None, spaceTimeDescriptor)
		self.func = partial(func, self.spaceTimeSet)
		self.type: LambdaType = lambdaType

	def __repr__(self):
		return "λ(%s, %s)" % (self._originalFuncName, self.type.name)
class Prototypes:
	"""
	Each object literal represents one spec's validators

	Each validator receives two arguments:
	* first argument is the space-time of interest for this validator, and
	* the second one is the space-time of the NFA
	"""
	P2 = None
	TET1 = None
	@staticmethod
	def initialize():
		Prototypes.P2 = ObjectLiteral(
			# A = Lambda(Prototypes.funcP2A, [[29, 46], [56, 46], [56, -1], [29, -1]], LambdaType.Region),
			# B = Lambda(Prototypes.funcP2B, [[19, 201], [41, 201], [41, 154], [19, 154]], LambdaType.Region),
			# C = Lambda(Prototypes.funcP2C, [[180, 140], [195, 140], [195, 160], [180, 160]], LambdaType.Region),
			A = Lambda(Prototypes.funcP2A, [[40, 30], [60, 30], [60, 20], [40, 20]], LambdaType.Region),
			B = Lambda(Prototypes.funcP2B, [[30, 180], [50, 180], [50, 170], [30, 170]], LambdaType.Region),
			C = Lambda(Prototypes.funcP2C, [[180, 140], [195, 140], [195, 160], [180, 160]], LambdaType.Region),
			T0 = Lambda(Prototypes.funcP2T0, 10, LambdaType.Time)
		)
		Prototypes.TET1 = ObjectLiteral(
			A = Lambda(Prototypes.funcP2A, [[10, 40], [20, 40], [20, 30], [10, 30]], LambdaType.Region),
			B = Lambda(Prototypes.funcP2B, [[90, 40], [80, 40], [80, 30], [90, 30]], LambdaType.Region),
			T0 = Lambda(Prototypes.funcP2T0, 10, LambdaType.Time)
		)

	@staticmethod
	def funcP2A(mySpaceTime: SpaceTimeSet, spaceTimeOfQuery: SpaceTimeSet):
		return Prototypes.funcIntersectionCheck(mySpaceTime.polygon, spaceTimeOfQuery.polygon)

	@staticmethod
	def funcP2B(mySpaceTime: SpaceTimeSet, spaceTimeOfQuery: SpaceTimeSet):
		return Prototypes.funcIntersectionCheck(mySpaceTime.polygon, spaceTimeOfQuery.polygon)

	@staticmethod
	def funcP2C(mySpaceTime: SpaceTimeSet, spaceTimeOfQuery: SpaceTimeSet):
		return Prototypes.funcIntersectionCheck(mySpaceTime.polygon, spaceTimeOfQuery.polygon)

	@staticmethod
	def funcIntersectionCheck(poly1, poly2):
		intersection = Geometry.intersect(poly1, poly2)
		try:
			return intersection.area > 0
		except:
			return intersection[0].area > 0
		return False

	@staticmethod
	def funcP2T0(mySpaceTime: SpaceTimeSet, spaceTimeOfQuery: SpaceTimeSet):
		return spaceTimeOfQuery.timeRange > mySpaceTime.timeRange
