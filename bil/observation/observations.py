from typing import Dict
from bil.observation.pose import Pose
from bil.model.trajectory import Trajectory

class Observation:
	def __init__(self, time, sensors, tracks):
		self.time = time
		self.sensors = sensors
		self.tracks = tracks

	def __repr__(self):
		return "Obs-%s" % repr(self.time)

class Observations:
	def __init__(self):
		self._dict: Dict[float, Observation] = {}
		self.timeArray = []
		self._trajectories = None
		self._fieldOfView = None

	def __len__(self):
		return len(self._dict)

	def __repr__(self):
		return repr(self._dict)

	def __iter__(self):
		yield from self._dict

	def __getitem__(self, key) -> Observation:
		return self._dict[key]

	@property
	def trajectories(self):
		if self._trajectories is not None: return self._trajectories
		poses = []
		for obsInd in self:
			if len(self[obsInd].tracks) == 0: continue
			observation = self[obsInd]
			for trackId in observation.tracks:
				track = observation.tracks[trackId]
				poses.append(track.pose)
		self._trajectories = [Trajectory("Traj-1", poses)]
		return self._trajectories

	def addObservation(self, observation: Observation):
		if observation.time in self._dict: raise RuntimeError("Repeated observation timestamp.")
		self.timeArray.append(observation.time)
		self._dict[observation.time] = observation
