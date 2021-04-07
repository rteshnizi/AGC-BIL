from shapely.geometry import Point

class Pose:
	def __init__(self, time, x, y, angleFromX):
		self.time: float = time
		self.x: float = x
		self.y: float = y
		self.angleFromX: float = angleFromX
		self.pt = Point(x, y)
		# FIXME: Set when deletedTracks is set and when new ID is detected in tracks dict
		self.spawn = False
		self.vanished = False

	def __repr__(self):
		if self.spawn and self.vanished: return "%s(%.2f, %.2f, %.2f)" % ("+/- ", self.time, self.x, self.y)
		if self.spawn: return "%s(%.2f, %.2f, %.2f)" % ("+ ", self.time, self.x, self.y)
		if self.vanished: return "%s(%.2f, %.2f, %.2f)" % ("- ", self.time, self.x, self.y)
		return "(%.2f, %.2f, %.2f)" % (self.time, self.x, self.y)
