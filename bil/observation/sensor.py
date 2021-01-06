from bil.model.trajectory import Trajectory
from bil.observation.pose import Pose

class Sensor:
	def __init__(self, idNum, time, x, y, psi, region):
		self.id = idNum
		self.pose = Pose(time, x, y, psi)
		self.region = region

	def __repr__(self):
		return "Sensor%d" % self.id
