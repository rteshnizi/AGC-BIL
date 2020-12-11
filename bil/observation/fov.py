from bil.observation.pose import Pose

class FOV:
	def __init__(self, time, x, y, psi, region):
		self.pose = Pose(time, x, y, psi)
		self.region = region

	def __repr__(self):
		return "FOV-%s" % repr(self.pose)
