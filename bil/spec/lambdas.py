from enum import Enum
from functools import partial

from bil.observation.pose import Pose
from bil.utils.geometry import Geometry
from bil.utils.python import ObjectLiteral

class LambdaType(Enum):
	Region = 1
	Time = 2

class Lambda:
	def __init__(self, func, spaceTimeSet, lambdaType: LambdaType):
		self._originalFuncName = func.__name__
		self.func = partial(func, spaceTimeSet)
		self.spaceTimeSet = spaceTimeSet
		self.type: LambdaType = lambdaType

	def __repr__(self):
		return "λ(%s, %s)" % (self._originalFuncName, self.type.name)

class Prototypes:
	P2 = None
	@staticmethod
	def initialize():
		Prototypes.P2 = ObjectLiteral(
			A = Lambda(Prototypes.funcP2A, [[29, 46], [56, 46], [56, -1], [29, -1]], LambdaType.Region),
			B = Lambda(Prototypes.funcP2B, [[19, 201], [41, 201], [41, 154], [19, 154]], LambdaType.Region),
			C = Lambda(Prototypes.funcP2C, [[180, 140], [195, 140], [195, 160], [180, 160]], LambdaType.Region),
			T0 = Lambda(Prototypes.funcP2T0, None, LambdaType.Time)
		)

	@staticmethod
	def funcP2A(spaceTime, pose: Pose):
		# return Geometry.isInside(pose, [[31, 44], [54, 44], [54, 1], [31, 1]])
		return False

	@staticmethod
	def funcP2B(spaceTime, pose: Pose):
		# return Geometry.isInside(p, [[21, 199], [39, 199], [39, 156], [21, 156]])
		return False

	@staticmethod
	def funcP2C(spaceTime, pose: Pose):
		return False

	@staticmethod
	def funcP2T0(spaceTime, pose: Pose):
		return True


###### LAM
