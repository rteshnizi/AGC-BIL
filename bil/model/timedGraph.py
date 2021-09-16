from bil.model.sensingRegion import SensingRegion
import networkx as nx
from shapely.geometry import LineString, Point, Polygon
from skimage import transform
import time
from typing import List, Set, Dict, Tuple
from math import inf

from bil.model.connectivityGraph import ConnectivityGraph
from bil.utils.geometry import Geometry
from bil.utils.graph import GraphAlgorithms
from bil.model.map import Map
from bil.utils.priorityQ import PriorityQ

class TimedGraph(nx.DiGraph):
	def __init__(self, graphs: List[ConnectivityGraph], startInd = 0, endInd = None):
		super().__init__()
		self.MIN_TIME_DELTA = 0.01
		self._nodeLayers = []
		self._timeStamps = []
		self._regionNodes = []
		self._beamNodes = []
		self._beamSet = set()
		self._boundary = None
		self.red = []
		self.blue = []
		print("Connecting Graphs through time...")
		self._fig = None
		self.nodeClusters: Dict[str, Set[str]] = {}
		self.nodeToClusterMap: Dict[str, str] = {}
		self._build(graphs, startInd, endInd)

	def _getLowerAndUpperNode(self, n1, n2):
		upper = n1 if self.nodes[n1]["timestamp"] > self.nodes[n2]["timestamp"] else n2
		lower = n1 if upper != n1 else n2
		return (lower, upper)

	def _addTemporalEdge(self, n1, n2):
		(lower, upper) = self._getLowerAndUpperNode(n1, n2)
		self.add_edge(lower, upper, isTemporal=True)

	def _checkChangesInCollidingEdges(self, previousFovPolygon: Polygon, currentFovPolygon: Polygon, envMap: Map) -> Tuple[bool, Set[Geometry.CoordsList], Set[Geometry.CoordsList]]:
		"""
			Takes two connectivity graphs and looks at their respective FOVs
			to see if the edges of the FOV that collides crosses a different set of edges of the map's boundary.

			#### Return
			A tuple with three elements:
				(Whether the edges are the same or neighboring,
				The edges hitting the previous FOV configuration,
				The edges hitting the current FOV configuration)
		"""
		fovVerts = list(previousFovPolygon.exterior.coords)
		previousEdges = set()
		for v1, v2 in zip(fovVerts, fovVerts[1:]):
			line = LineString([v1, v2])
			edges = Geometry.getAllIntersectingEdges(line, envMap.polygon)
			previousEdges.update(edges)
		fovVerts = list(currentFovPolygon.exterior.coords)
		currentEdges = set()
		for v1, v2 in zip(fovVerts, fovVerts[1:]):
			line = LineString([v1, v2])
			edges = Geometry.getAllIntersectingEdges(line, envMap.polygon)
			currentEdges.update(edges)
		prevHash = [Geometry.coordListStringId(edge) for edge in previousEdges]
		currHash = [Geometry.coordListStringId(edge) for edge in currentEdges]
		edgesAreTheSame = prevHash == currHash
		if edgesAreTheSame: return (True, previousEdges, currentEdges)
		# # Are they almost the same (neighboring)
		# for edge1coords in previousEdges:
		# 	pts1Set = set(edge1coords)
		# 	for edge2coords in currentEdges:
		# 		pts2Set = set(edge2coords)
		# 		coordsInCommon = pts1Set.intersection(pts2Set)
		# 		# Common edge doesn't count
		# 		if len(coordsInCommon) == 0:
		# 			continue
		# 		if len(coordsInCommon) == 2:
		# 			break
		# 		if len(coordsInCommon) == 1:
		# 			# There is a neghboring edge
		# 			return (True, previousEdges, currentEdges)
		# 		raise("Past Reza said, check when does this happen")
		return (False, previousEdges, currentEdges)

	def _getAffineTransformation(self, previousFovPolygon: Polygon, currentFovPolygon: Polygon):
		matrix = Geometry.getAffineTransformation(previousFovPolygon.exterior.coords, currentFovPolygon.exterior.coords)
		return matrix

	def _getCollidingEdgesByEdge(self, sensor: SensingRegion, envMap: Map) -> Dict[str, Set[LineString]]:
		collisionData = {}
		for sensorEdgeId in sensor.edges:
			collisionData[sensorEdgeId] = Geometry.getAllIntersectingEdges(sensor.edges[sensorEdgeId], envMap.polygon)
		return collisionData

	def _checkIntervalForCollision(self, movingEdge: LineString, staticEdge: LineString, transformation: transform.AffineTransform, intervalStart: float, intervalEnd: float) -> Tuple[bool, bool]:
		startConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalStart)
		endConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalEnd)
		movingEdgeStartConfiguration = Geometry.applyMatrixTransformToLineString(startConfigTransformation, movingEdge)
		movingEdgeEndConfiguration = Geometry.applyMatrixTransformToLineString(endConfigTransformation, movingEdge)
		return (movingEdgeStartConfiguration.intersects(staticEdge), movingEdgeEndConfiguration.intersects(staticEdge))

	def _checkIntervalsForOverlap(self, interval1: Tuple[float, float], interval2: Tuple[float, float]) -> bool:
		# If end of one interval happens earlier than the other
		if interval1[1] <= interval2[0]: return False
		if interval2[1] <= interval1[0]: return False
		return True

	def _splitIntervalsListForOverlap(self, intervals: List[Tuple[LineString, LineString, float, float]]) -> Tuple[List[Tuple[LineString, LineString, float, float]], List[Tuple[LineString, LineString, float, float]]]:
		haveOverlap = []
		dontHaveOverlap = []
		for interval1 in intervals:
			for interval2 in intervals:
				if self._checkIntervalsForOverlap(interval1[2:], interval2[2:]):
					foundOverlap = True
					haveOverlap.append(interval1)
					break
			if not foundOverlap:
				dontHaveOverlap.append(interval1)
		return (haveOverlap, dontHaveOverlap)

	def _findIntermediateComponentEvents(self, previousSensor: SensingRegion, currentSensor: SensingRegion, envMap: Map, queryStartTime = 0, queryEndTime = 1) -> List[Polygon]:
		"""
			Given the original configuration of the FOV (`previousSensor`) and the final configuration (`currentSensor`)
			find all the times where there is a shadow component event.

			That is, as the FOV moves towards its final configuration save the intermediate configurations for which there is a topological change (FOV hitting a "gap").
			#### Returns
			A list of intermediate between which there is at most one component event.
		"""
		transformation = self._getAffineTransformation(previousSensor.polygon, currentSensor.polygon)
		previousCollidingEdges = self._getCollidingEdgesByEdge(previousSensor, envMap)
		currentCollidingEdges = self._getCollidingEdgesByEdge(currentSensor, envMap)
		# This Priority Queue holds a list of intervals.
		# The list is ordered from earliest to latest time the edges are in contact with the polygon's edge.
		# Each item in the pQ is a tuple: (time, sensorEdge, staticEdge)
		# keyFunc = lambda x: x[0]
		# pQ = PriorityQ(keyFunc)
		intervals = []
		for previousSensorEdgeId in previousCollidingEdges:
			staticEdges = previousCollidingEdges[previousSensorEdgeId]
			if (len(staticEdges) == 0): continue
			movingEdge = previousSensor.edges[previousSensorEdgeId]
			for staticEdge in staticEdges:
				collsionCheckResults = self._checkIntervalForCollision(
					movingEdge=movingEdge,
					staticEdge=staticEdge,
					transformation=transformation,
					intervalStart=queryStartTime,
					intervalEnd=queryEndTime)
				if collsionCheckResults[0] == collsionCheckResults[1]: continue
				intervals.append((movingEdge, staticEdge, queryStartTime, queryEndTime))
		haveOverlap = intervals
		dontHaveOverlap = []
		while len(haveOverlap) > 0:
			(movingEdge, staticEdge, intervalStart, intervalEnd) = haveOverlap.pop(0)
			collsionCheckResults = self._checkIntervalForCollision(
				movingEdge=movingEdge,
				staticEdge=staticEdge,
				transformation=transformation,
				intervalStart=intervalStart,
				intervalEnd=intervalEnd)
			if collsionCheckResults[0] == collsionCheckResults[1]: continue
			midInterval = (intervalStart + intervalEnd) / 2
			if midInterval == intervalStart and midInterval == intervalEnd:
				i = 0
			haveOverlap.append((movingEdge, staticEdge, intervalStart, midInterval))
			haveOverlap.append((movingEdge, staticEdge, midInterval, intervalEnd))
			(haveOverlap, dontHaveOverlap) = self._splitIntervalsListForOverlap(haveOverlap)
		return []
		for currentSensorEdgeId in currentCollidingEdges:
			staticEdges = currentCollidingEdges[currentSensorEdgeId]
			if (len(staticEdges) == 0): continue
			movingEdgeFinalConfig = currentSensor.edges[currentSensorEdgeId]
			movingEdge = previousSensor.getEquivalentEdge(movingEdgeFinalConfig, transformation)
			if movingEdge is None: raise("Unexpected, based on this transformation there must be an edge.")
			for staticEdge in staticEdges:
				time = Geometry.findTheEarliestTimeTheyAreColliding(movingEdge, staticEdge, transformation)
				pQ.enqueue((time, currentSensorEdgeId, staticEdge))
				intermidiateTransform = Geometry.getParameterizedAffineTransformation(transformation, time)
				intermidiatePolygon = Geometry.applyMatrixTransformToPolygon(intermidiateTransform, previousSensor.polygon)
				# self.blue.append(intermidiatePolygon)
		return []

	def _build(self, graphs: List[ConnectivityGraph], startInd = 0, endInd = None):
		if endInd is None: endInd = len(graphs)

		self.nodesByLayer = []
		previousLayer = None
		startTime = time.time()
		for i in range(startInd, endInd):
			currentLayer = graphs[i]
			if previousLayer is not None:
				for sensorId in previousLayer.fov.sensors:
					self._findIntermediateComponentEvents(
						previousLayer.fov.sensors[sensorId].region,
						currentLayer.fov.getEquivalentSensorById(sensorId).region,
						previousLayer.map)
				return

			layerIndex = len(self.nodesByLayer)
			self.nodesByLayer.append([])
			self._timeStamps.append(currentLayer.timestamp)
			print ("Chaining for %.1f" % currentLayer.timestamp)
			gDense = currentLayer.condense()
			for n in gDense.nodes:
				newNode = GraphAlgorithms._getTimedNodeName(n, currentLayer.timestamp)
				self.add_node(newNode)
				n = n if n in currentLayer.nodes else gDense.nodes[n]["mappedName"]
				GraphAlgorithms.cloneNodeProps(currentLayer.nodes[n] if not n.startswith("sym") else gDense.nodes[n], self.nodes[newNode])
				self.nodes[newNode]["polygonName"] = n # This is lazy way to obtain the name of the polygon without doing string split etc.
				self.nodes[newNode]["cluster"] = currentLayer.nodeClusters[currentLayer.nodeToClusterMap[n]]
				self.nodesByLayer[layerIndex].append(newNode)
			for e in gDense.edges:
				self.add_edge(GraphAlgorithms._getTimedNodeName(e[0], currentLayer.timestamp), GraphAlgorithms._getTimedNodeName(e[1], currentLayer.timestamp), isTemporal=False)
			if layerIndex == 0:
				previousLayer = currentLayer
				continue
			# CREATE TEMPORAL EDGES
			for n1 in self.nodesByLayer[layerIndex - 1]:
				if GraphAlgorithms.isBeamNode(n1): continue
				for n2 in self.nodesByLayer[layerIndex]:
					if GraphAlgorithms.isBeamNode(n2): continue
					if self.nodes[n1]["type"] == "sensor" and self.nodes[n2]["type"] != "sensor": continue
					if self.nodes[n1]["type"] == "sensor" and self.nodes[n2]["type"] == "sensor":
						if self.nodes[n1]["polygonName"] != self.nodes[n2]["polygonName"]: continue
						self._addTemporalEdge(n1, n2)
						break
					if not Geometry.polygonAndPolygonIntersect(self.nodes[n1]["cluster"].polygon, self.nodes[n2]["cluster"].polygon): continue
					intersection = Geometry.intersect(self.nodes[n1]["cluster"].polygon, self.nodes[n2]["cluster"].polygon)[0] # We are sure there will be one intersection
					(pXs, pYs) = intersection.exterior.coords.xy
					connectedTemporally = True
					for sensorKey in currentLayer.fov.sensors:
						currentLayerSensor = currentLayer.fov.sensors[sensorKey]
						if not Geometry.haveOverlappingEdge(currentLayerSensor.region.polygon, intersection): continue
						for i in range(-1, len(currentLayerSensor._originalCoords)):
							(x1, y1) = currentLayerSensor._originalCoords[i]
							(x2, y2) = currentLayerSensor._originalCoords[i + 1]
							(x3, y3) = fovCoordsMap[(x1, y1)]
							(x4, y4) = fovCoordsMap[(x2, y2)]
							for j in range(len(pXs)):
								if self._testConnectivity(x1, y1, x2, y2, x3, y3, x4, y4, pXs[j], pYs[j]):
									# This means the instersection area was swept away by a sensor
									connectedTemporally = False
								if not connectedTemporally: break
							if not connectedTemporally: break
						if not connectedTemporally: continue
						self._addTemporalEdge(n1, n2)
					self._oldAlg(n1 ,n2)
			previousLayer = currentLayer
		endTime = time.time()
		print("%.3f s" % (endTime - startTime))

	def _oldAlg(self, n1, n2):
		shouldConnect = False
		if self.nodes[n1]["polygonName"] == self.nodes[n2]["polygonName"]: shouldConnect = True
		if not shouldConnect and self.nodes[n1]["polygonName"] in self.nodes[n1]["cluster"].nodes: shouldConnect = True
		if not shouldConnect: return
		self._addTemporalEdge(n1, n2)

	def getTemporalNeighbors(self, node: str, nodeLayerIndex: int = 0):
		timedName = GraphAlgorithms._getTimedNodeName(node, self._timeStamps[nodeLayerIndex])
		neighbors = set()
		for outEdge in self.out_edges(timedName):
			neighbor = outEdge[1]
			if not self.get_edge_data(outEdge[0], outEdge[1])["isTemporal"]: continue
			neighbors.add(neighbor)
		return neighbors

	def displayGraph(self, displayGeomGraph, displaySpringGraph):
		if displayGeomGraph:
			self._fig = GraphAlgorithms.displayGeometricGraph(self, self._regionNodes, self._beamNodes)
		else:
			self._fig = GraphAlgorithms.displaySpringGraph(self, self._regionNodes, self._beamNodes)

	def killDisplayedGraph(self):
		if self._fig:
			GraphAlgorithms.killDisplayedGraph(self._fig)
			self._fig = None
