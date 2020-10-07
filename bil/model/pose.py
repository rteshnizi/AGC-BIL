from shapely.geometry import Point

class Pose:
	def __init__(self, time, x, y, angleFromX):
		self.time = time
		self.x = x
		self.y = y
		self.angleFromX = angleFromX
		self.pt = Point(x, y)
