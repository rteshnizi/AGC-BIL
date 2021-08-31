from typing import Dict, Tuple
from bil.observation.track import Track
from bil.model.trajectory import Trajectory

class Observation:
	def __init__(self, time, fov, tracks):
		self.time = time
		self.fov = fov
		# Key is (time, trackId)
		self.tracks: Dict[Tuple[float, int], Track] = tracks

	def __repr__(self):
		return "Obs-%s" % repr(self.time)

class Observations:
	def __init__(self):
		self._dict: Dict[float, Observation] = {}
		self.timeArray = []
		self._trajectories = None

	def __len__(self):
		return len(self._dict)

	def __repr__(self):
		return repr(self._dict)

	def __iter__(self):
		yield from self._dict

	def __getitem__(self, key) -> Observation:
		return self._dict[key]

	def getObservationByIndex(self, index):
		return self._dict[self.timeArray[index]]

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
		self.timeArray.sort()
		self._dict[observation.time] = observation