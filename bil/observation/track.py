# from bil.model.trajectory import Trajectory
from bil.observation.pose import Pose

class Track:
	def __init__(self, idNum, time, x, y, psi):
		self.id = idNum
		self.pose = Pose(time, x, y, psi)

	def __repr__(self):
		return "Track%d: %s" % (self.id, repr(self.pose))
