from bil.model.trajectory import Trajectory
from bil.observation.pose import Pose

class Sensor:
	def __init__(self, idNum, fov):
		self.id = idNum
		self.fov = fov

	def __repr__(self):
		return "Sensor%d" % self.id
