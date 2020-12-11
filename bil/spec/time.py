from bil.observation.pose import Pose

class TimeSpecifier:
	def __init__(self, endPoints: list, intervalFlags: list, endPointFlags: list):
		self.endPoints = endPoints
		self.intervalFlags = intervalFlags
		self.endPointFlags = endPointFlags

	def __contains__(self, time: float):
		raise RuntimeError("Not implemented")

class Time:
	@staticmethod
	def query(p: Pose, timeSpecifier: TimeSpecifier) -> bool:
		return p.time in timeSpecifier
