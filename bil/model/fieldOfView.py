from typing import List, Dict
import networkx as nx

from bil.utils.graph import GraphAlgorithms
from bil.model.sensingRegion import SensingRegion
from bil.model.connectivityGraph import ConnectivityGraph
from bil.model.timedGraph import TimedGraph

class FieldOfView:
	def __init__(self, envMap):
		self.regions: Dict[float, SensingRegion] = {}
		# Index to timestamp
		self._timestamps = {}
		self.map = envMap
		self._cGraphs: List[ConnectivityGraph] = None
		self.condensedCGraphs: List[ConnectivityGraph] = []
		self._chainedGraphThroughTime: TimedGraph = None

	def __len__(self):
		return len(self.regions)

	def __getitem__(self, ind: int):
		return self.regions[self._timestamps[ind]]

	@property
	def cGraphs(self) -> List[ConnectivityGraph]:
		if self._cGraphs is None:
			self._cGraphs = []
			index = 0
			for (t, regionList) in self.regions.items():
				self._cGraphs.append(ConnectivityGraph(self.map, regionList, t, index))
				index += 1
		return self._cGraphs

	def chainedGraphThroughTime(self, spec):
		if self._chainedGraphThroughTime is None:
			self._chainedGraphThroughTime = TimedGraph(self.cGraphs, spec)
		return self._chainedGraphThroughTime

	def append(self, region, timestamp, agentIndex):
		if timestamp not in self.regions:
			self._timestamps[len(self._timestamps)] = timestamp
			self.regions[timestamp] = []
		self.regions[timestamp].append(SensingRegion("F%d" % (len(self.regions[timestamp])), region, timestamp, agentIndex))

	def render(self, canvas):
		for (_, regionList) in self.regions.items():
			for region in regionList:
				region.render(canvas, False)

	def clearRender(self, canvas):
		for (_, regionList) in self.regions.items():
			for region in regionList:
				region.clearRender(canvas)
