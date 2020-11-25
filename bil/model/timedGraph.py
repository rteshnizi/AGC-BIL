import networkx as nx
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point, Polygon
from typing import List, Set, Dict
import time

from bil.model.observationOld import ObservationOld
from bil.model.polygonalRegion import PolygonalRegion
from bil.model.shadowRegion import ShadowRegion
from bil.model.connectivityGraph import ConnectivityGraph
from bil.utils.geometry import Geometry
from bil.utils.graph import GraphAlgorithms

class TimedGraph(nx.DiGraph):
	def __init__(self, graphs: List[ConnectivityGraph], spec, startInd = 0, endInd = None):
		super().__init__()
		self._nodeLayers = []
		self._regionNodes = []
		self._beamNodes = []
		self._beamSet = set()
		print("Connecting Graphs through time...")
		self._fig = None
		self.nodeClusters: Dict[str, Set[str]] = {}
		self.nodeToClusterMap: Dict[str, str] = {}
		self._build(graphs, spec, startInd, endInd)

	def _build(self, graphs: List[ConnectivityGraph], spec, startInd = 0, endInd = None):
		if endInd is None: endInd = len(graphs)
		if (startInd - endInd) >= 0:
			startInd = 0
			endInd = len(graphs)

		self.nodeLayers = []
		for i in range(startInd, endInd):
			g = graphs[i]
			layerIndex = len(self.nodeLayers)
			self.nodeLayers.append([])
			print ("Chaining for %.1f" % g.timestamp)
			gDense = g.condense(spec)
			for n in gDense.nodes:
				newNode = GraphAlgorithms._getTimedNodeName(n, g.timestamp)
				self.add_node(newNode)
				n = n if n in g.nodes else gDense.nodes[n]["mappedName"]
				GraphAlgorithms.cloneNodeProps(g.nodes[n], self.nodes[newNode])
				self.nodes[newNode]["polygonName"] = n # This is lazy way to obtain the name of the polygon without doing string split etc.
				self.nodes[newNode]["cluster"] = g.nodeClusters[g.nodeToClusterMap[n]]
				self.nodeLayers[layerIndex].append(newNode)
			for e in gDense.edges:
				self.add_edge(GraphAlgorithms._getTimedNodeName(e[0], g.timestamp), GraphAlgorithms._getTimedNodeName(e[1], g.timestamp))

		print("n = %d" % len(self.nodes))
		print("O(%d)" % (len(self.nodes) * len(self.nodes)))
		startTime = time.time()
		for layerIndex in range(len(self.nodeLayers) - 1):
			lowerLayer = self.nodeLayers[layerIndex]
			upperLayer = self.nodeLayers[layerIndex + 1]
			for n1 in lowerLayer:
				for n2 in upperLayer:
					shouldConnect = False
					if self.nodes[n1]["polygonName"] == self.nodes[n2]["polygonName"]: shouldConnect = True
					if not shouldConnect and self.nodes[n1]["polygonName"] in self.nodes[n1]["cluster"]: shouldConnect = True
					if not shouldConnect: continue
					higher = n1 if self.nodes[n1]["timestamp"] > self.nodes[n2]["timestamp"] else n2
					lower = n1 if higher != n1 else n2
					self.add_edge(lower, higher)

		# All Layers
		# for n1 in self.nodes:
		# 	for n2 in self.nodes:
		# 		if self.nodes[n1]["polygonName"] != self.nodes[n2]["polygonName"]: continue
		# 		higher = n1 if self.nodes[n1]["timestamp"] > self.nodes[n2]["timestamp"] else n2
		# 		lower = n1 if higher != n1 else n2
		# 		self.add_edge(lower, higher)
		endTime = time.time()
		print("%.3f s" % (endTime - startTime))

	def displayGraph(self, displayGeomGraph, displaySpringGraph):
		if displayGeomGraph:
			self._fig = GraphAlgorithms.displayGeometricGraph(self, self._regionNodes, self._beamNodes)
		else:
			self._fig = GraphAlgorithms.displaySpringGraph(self, self._regionNodes, self._beamNodes)

	def killDisplayedGraph(self):
		if self._fig:
			GraphAlgorithms.killDisplayedGraph(self._fig)
			self._fig = None
