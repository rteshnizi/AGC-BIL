from bil.spec.time import TimeSpecifier

class SpaceTimeSet:
	# FIXME: Use TimeRange instead of float. But for now we are projecting
	def __init__(self, shapelyPolygon, timeRange: float):
		self.polygon = shapelyPolygon
		self.timeRange = timeRange
