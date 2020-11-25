from typing import Dict
from bil.model.pose import Pose

class Observation:
	def __init__(self):
		self.fov = {}
		self.tracks = {}
		self.deletedTracks = []

	def __repr__(self):
		return "%s - %s" % (repr(self.fov), repr(self.tracks))

	def addTrack(self, trackId, pose):
		self.tracks[trackId] = pose

	def addFov(self, fovId, regions, cGraph):
		self.fov[fovId] = (regions, cGraph)

class Observations:
	def __init__(self):
		self._dict: Dict[float, Observation] = {}

	def __iter__(self):
		yield from self._dict

	def __getitem__(self, key):
		return self._dict[key]

	def appendTrack(self, trackId, poseData):
		pose = Pose(poseData[0], poseData[1], poseData[2], poseData[3])
		if pose.time not in self._dict:
			self._dict[pose.time] = Observation()
		self._dict[pose.time].addTrack(trackId, pose)

	def mapFov(self, fov):
		for i in range(len(fov)):
			sensingRegions = fov[i]
			if len(sensingRegions) == 0: continue
			if sensingRegions[0].timestamp not in self._dict:
				self._dict[sensingRegions[0].timestamp] = Observation()
			self._dict[fov.cGraphs[i].timestamp].addFov(fov.cGraphs[i].timestamp, sensingRegions, fov.cGraphs[i])
