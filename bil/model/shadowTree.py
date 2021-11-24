from bil.model.sensingRegion import SensingRegion
import networkx as nx
from shapely.geometry import LineString, Point, Polygon
from skimage import transform
import time
from typing import List, Set, Dict, Tuple

from bil.model.connectivityGraph import ConnectivityGraph
from bil.model.map import Map
from bil.observation.fov import Fov
from bil.spec.validator import Validator
from bil.utils.geometry import Geometry
from bil.utils.graph import GraphAlgorithms
from bil.utils.priorityQ import PriorityQ

class ShadowTree(nx.DiGraph):
	def __init__(self, envMap: Map, fovs: List[Fov], validators: Dict[str, Validator], startInd = 0, endInd = None):
		super().__init__()
		self.MIN_TIME_DELTA = 1E-2
		self._fig = None
		# The reason this is a list of lists is that the time of event is relative to the time between
		self.componentEvents: List[List[Polygon]] = []
		self.graphs: List[ConnectivityGraph] = []
		# Used for debugging
		self.redPolys: List[Polygon] = []
		self.bluePolys: List[Polygon] = []
		self.lines: List[LineString] = []

		print("Connecting Graphs through time...")
		self._build(envMap, fovs, validators, startInd, endInd)

	def __repr__(self) -> str:
		return "ShadowTree"

	def _generateTemporalName(self, name: str, time: float):
		return "%s-%.2f" % (name, time)

	def _getLowerAndUpperNode(self, n1, n2):
		upper = n1 if self.nodes[n1]["fromTime"] > self.nodes[n2]["fromTime"] else n2
		lower = n1 if upper != n1 else n2
		return (lower, upper)

	def _addNode(self, n, time):
		self.add_node(n)
		self.nodes[n]["fromTime"] = time
		self.nodes[n]["toTime"] = time # This will be updated accordingly when adding temporal edges

	def _addEdge(self, n1: str, n2: str, isTemporal: bool, fromTime = None, toTime = None):
		(lower, upper) = self._getLowerAndUpperNode(n1, n2)
		self.add_edge(lower, upper, isTemporal=isTemporal, fromTime=fromTime, toTime=toTime)

	def _shadowsAreConnectedTemporally(self, previousGraph: ConnectivityGraph, currentGraph: ConnectivityGraph, previousShadow: dict, currentShadow: dict, centerOfRotation: Geometry.Coords):
		"""
			With the assumption that previousNode and currentNode intersect,
			 1. takes the intersection
			 2. finds the polygon made by the translation of each edge
			 3. takes the union of those polygons to get the area swept by FOV
			 4. if the intersection has areas that are not swept by FOV, then they are connected
		"""
		previousP: Polygon = previousShadow["region"].polygon
		currentP: Polygon = currentShadow["region"].polygon
		intersectionOfShadows: Polygon = previousP.intersection(currentP)
		for fov in previousGraph.fovNodes:
			previousFovRegion: SensingRegion = previousGraph.nodes[fov]["region"]
			currentFovRegion: SensingRegion = currentGraph.nodes[fov]["region"]
			transformation: transform.AffineTransform = Geometry.getAffineTransformation(previousFovRegion.polygon, currentFovRegion.polygon, centerOfRotation)
			ps = []
			for e in currentFovRegion.edges:
				edgeC = currentFovRegion.edges[e]
				edgeP = previousFovRegion.getEquivalentEdge(edgeC, transformation, centerOfRotation)
				polygon = Polygon([edgeP.coords[0], edgeP.coords[1], edgeC.coords[1], edgeC.coords[0]])
				ps.append(polygon)
			u = Geometry.union(ps)
			p = intersectionOfShadows.difference(u)
			if p.area > 0.01:
				return True
		return False

	def _getCollidingEdgesByEdge(self, sensor: SensingRegion, envMap: Map) -> Dict[str, Set[LineString]]:
		collisionData = {}
		for sensorEdgeId in sensor.edges:
			collisionData[sensorEdgeId] = Geometry.getAllIntersectingEdgesWithLine(sensor.edges[sensorEdgeId], envMap.polygon)
		return collisionData

	def _checkEdgeIntervalForCollision(self, movingEdge: LineString, staticEdge: LineString, transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords, intervalStart: float, intervalEnd: float) -> Tuple[bool, bool]:
		startConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalStart)
		endConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalEnd)
		movingEdgeStartConfiguration = Geometry.applyMatrixTransformToLineString(startConfigTransformation, movingEdge, centerOfRotation)
		movingEdgeEndConfiguration = Geometry.applyMatrixTransformToLineString(endConfigTransformation, movingEdge, centerOfRotation)
		return (movingEdgeStartConfiguration.intersects(staticEdge), movingEdgeEndConfiguration.intersects(staticEdge))

	def _checkBoundingBoxIntervalForCollision(self, sensor: SensingRegion, movingEdge: LineString, staticEdge: LineString, transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords, intervalStart: float, intervalEnd: float) -> Tuple[Tuple[bool, bool, bool], Polygon, Polygon, LineString]:
		"""
			#### Returns
			Given the configuration information, it returns a tuple: `((bool, bool, bool), Polygon, Polygon)`
			1. A tuple of the state of collision in the interval: `(isCollidingAtStart, isCollidingAtMid, isCollidingAtEnd)`
			2. firstHalfBb
			3. secondHalfBb
		"""
		startConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalStart)
		edgeAtStart = Geometry.applyMatrixTransformToLineString(startConfigTransformation, movingEdge, centerOfRotation)
		isCollidingAtStart = Geometry.intersectLineSegments(edgeAtStart, staticEdge) is not None
		endConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalEnd)
		edgeAtEnd = Geometry.applyMatrixTransformToLineString(endConfigTransformation, movingEdge, centerOfRotation)
		isCollidingAtEnd = Geometry.intersectLineSegments(edgeAtEnd, staticEdge) is not None
		intervalMid = (intervalStart + intervalEnd) / 2
		midConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalMid)
		edgeAtMid = Geometry.applyMatrixTransformToLineString(midConfigTransformation, movingEdge, centerOfRotation)
		isCollidingAtMid = Geometry.intersectLineSegments(edgeAtMid, staticEdge) is not None
		startSensor = Geometry.applyMatrixTransformToPolygon(startConfigTransformation, sensor.polygon, centerOfRotation)
		midSensor = Geometry.applyMatrixTransformToPolygon(midConfigTransformation, sensor.polygon, centerOfRotation)
		endSensor = Geometry.applyMatrixTransformToPolygon(endConfigTransformation, sensor.polygon, centerOfRotation)
		firstHalfTransformation = Geometry.getAffineTransformation(startSensor, midSensor, centerOfRotation)
		secondHalfTransformation = Geometry.getAffineTransformation(midSensor, endSensor, centerOfRotation)
		firstHalfBb = self._getLineSegmentExpandedBb(firstHalfTransformation, edgeAtStart, firstHalfTransformation.rotation, centerOfRotation)
		secondHalfBb = self._getLineSegmentExpandedBb(secondHalfTransformation, edgeAtMid, secondHalfTransformation.rotation, centerOfRotation)
		return ((isCollidingAtStart, isCollidingAtMid, isCollidingAtEnd), firstHalfBb, secondHalfBb, edgeAtMid)

	def _checkIntervalsForOverlap(self, interval1: Tuple[float, float], interval2: Tuple[float, float]) -> bool:
		# If end of one interval happens earlier than the other
		if interval1[1] <= interval2[0]: return False
		if interval2[1] <= interval1[0]: return False
		return True

	def _edgesHaveACommonVertex(self, l1: LineString, l2: LineString):
		l1Verts = l1.coords
		l2Verts = l2.coords
		for v1 in l1Verts:
			for v2 in l2Verts:
				if Geometry.coordsAreAlmostEqual(v1, v2): return True
		return False

	def _splitIntervalsListForOverlap(self, intervals: List[Tuple[LineString, LineString, float, float]]) -> Tuple[List[Tuple[LineString, LineString, float, float]], List[Tuple[LineString, LineString, float, float]]]:
		haveOverlap = []
		dontHaveOverlap = []
		while len(intervals) > 0:
			interval1 = intervals.pop()
			foundOverlap = False
			for interval2 in intervals:
				if interval1 == interval2: continue
				if self._checkIntervalsForOverlap(interval1[2:], interval2[2:]):
					if interval1[0] == interval2[0]:
						# If the sensor edges are the same
						if self._edgesHaveACommonVertex(interval1[1], interval2[1]):
							# AND if the map edges are consecutive,
							# the sensor edge is going to collide both edges at the common vertex.
							# So we can safely remove one of them from the overlap
							continue
					foundOverlap = True
					haveOverlap.append(interval1)
					break
			if not foundOverlap:
				dontHaveOverlap.append(interval1)
		return (haveOverlap, dontHaveOverlap)

	def _expandVertObbWithAngularVelocity(self, coords: Geometry.Coords, angle: float, centerOfRotation: Geometry.Coords, expandAway = True):
		displacement = (coords[0] - centerOfRotation[0], coords[1] - centerOfRotation[1])
		vertExpansion = (angle * displacement[0], angle * displacement[1])
		expanded = (coords[0] + vertExpansion[0], coords[1] + vertExpansion[1]) if expandAway else (coords[0] - vertExpansion[0], coords[1] - vertExpansion[1])
		return expanded

	def _getLineSegmentExpandedBb(self, transformation: transform.AffineTransform, lineSeg: LineString, angle: float, centerOfRotation: Geometry.Coords) -> Polygon:
		"""
			Gets a tight bounding box for a line segment that is moving with a constant angular velocity.
		"""
		finalConfig = Geometry.applyMatrixTransformToLineString(transformation, lineSeg, centerOfRotation)
		polygons = []
		for j in range(16):
			v1 = self._expandVertObbWithAngularVelocity(lineSeg.coords[0], angle, centerOfRotation, j & 1 != 0)
			v2 = self._expandVertObbWithAngularVelocity(finalConfig.coords[0], angle, centerOfRotation, j & 2 != 0)
			v3 = self._expandVertObbWithAngularVelocity(finalConfig.coords[1], angle, centerOfRotation, j & 4 != 0)
			v4 = self._expandVertObbWithAngularVelocity(lineSeg.coords[1], angle, centerOfRotation, j & 8 != 0)
			p = Polygon([v1, v2, v3, v4])
			polygons.append(p)
		expandedObb = Geometry.union(polygons)
		expandedObb = expandedObb.convex_hull
		return expandedObb

	def _findColissionsWithExtendedBb(self, sensor: SensingRegion, transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords, envMap: Map) -> Dict[str, Set[LineString]]:
		"""
			#### Returns
				For each edge of the sensor, it returns all the edges of the map that intersect the expanded bounding box of that edge
		"""
		collisionData = {}
		angle = abs(transformation.rotation)
		for sensorEdgeId in sensor.edges:
			collisionData[sensorEdgeId] = []
			edge = sensor.edges[sensorEdgeId]
			boundingBox = self._getLineSegmentExpandedBb(transformation, edge, angle, centerOfRotation)
			mapBoundaryVerts = envMap.polygon.exterior.coords
			for v1, v2 in zip(mapBoundaryVerts, mapBoundaryVerts[1:]):
				mapEdge = LineString([v1, v2])
				if boundingBox.intersects(mapEdge):
					collisionData[sensorEdgeId].append(mapEdge)
		return collisionData

	def _initEdgeIntervals(self, sensor: SensingRegion, transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords, collisionData: Dict[str, Set[LineString]], inverse: bool):
		"""
			Initialize intervals for edges that are currently collising with the sensor.
		"""
		intervals = []
		intStart = 1 if inverse else 0
		intEnd = 0 if inverse else 1
		for sensorEdgeId in collisionData:
			mapEdges = collisionData[sensorEdgeId]
			if (len(mapEdges) == 0): continue
			sensorEdge = sensor.edges[sensorEdgeId]
			for mapEdge in mapEdges:
				collsionCheckResults = self._checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, centerOfRotation, intervalStart=intStart, intervalEnd=intEnd)
				if collsionCheckResults[0] == collsionCheckResults[1]: continue
				intervals.append((sensorEdge, mapEdge, 0, 1))
		return intervals

	def _initColissionIntervals(self, sensor: SensingRegion, collisionData: Dict[str, Set[LineString]]):
		intervals = []
		for sensorEdgeId in collisionData:
			sensorEdge = sensor.edges[sensorEdgeId]
			mapEdges = collisionData[sensorEdgeId]
			if (len(mapEdges) == 0): continue
			for mapEdge in mapEdges:
				intervals.append((sensorEdge, mapEdge, 0, 1))
		return intervals

	def _findEventIntervalsForCollisions(self, previousSensor: SensingRegion, collisionIntervals: Dict[str, Set[LineString]], transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords, ingoingIntervals: list, outgoingIntervals: list):
		i = 0
		while i < len(collisionIntervals):
			(sensorEdge, mapEdge, intervalStart, intervalEnd) = collisionIntervals.pop(i)
			# Epsilon for shard search
			deltaT = intervalEnd - intervalStart
			if deltaT <= self.MIN_TIME_DELTA:
				continue
			((isCollidingAtStart, isCollidingAtMid, isCollidingAtEnd), firstHalfBb, secondHalfBb, edgeAtMid) = self._checkBoundingBoxIntervalForCollision(previousSensor, sensorEdge, mapEdge, transformation, centerOfRotation, intervalStart, intervalEnd)
			intervalMid = (intervalStart + intervalEnd) / 2
			if isCollidingAtStart != isCollidingAtMid:
				if isCollidingAtStart and not isCollidingAtMid:
					outgoingIntervals.append((sensorEdge, mapEdge, intervalStart, intervalMid))
				else:
					ingoingIntervals.append((sensorEdge, mapEdge, intervalStart, intervalMid))
			if isCollidingAtMid != isCollidingAtEnd:
				if isCollidingAtMid and not isCollidingAtEnd:
					outgoingIntervals.append((sensorEdge, mapEdge, intervalStart, intervalMid))
				else:
					ingoingIntervals.append((sensorEdge, mapEdge, intervalStart, intervalMid))
			if isCollidingAtStart == isCollidingAtMid and isCollidingAtMid == isCollidingAtEnd:
				if firstHalfBb.intersects(mapEdge):
					collisionIntervals.insert(i, (sensorEdge, mapEdge, intervalStart, intervalMid))
					i += 1
				if secondHalfBb.intersects(mapEdge):
					collisionIntervals.insert(i, (sensorEdge, mapEdge, intervalMid, intervalEnd))
					i += 1
		return

	def _findIntermediateComponentEvents(self, previousSensor: SensingRegion, currentSensor: SensingRegion, centerOfRotation: Geometry.Coords, envMap: Map) -> Tuple[Polygon, float, str, LineString]:
		"""
			Given the original configuration of the FOV (`previousSensor`) and the final configuration (`currentSensor`)
			find all the times where there is a shadow component event.

			That is, as the FOV moves towards its final configuration save the intermediate configurations for which there is a topological change (FOV hitting a "gap").
			#### Returns
			A list of intermediate between which there is at most one component event.
		"""
		transformation = Geometry.getAffineTransformation(previousSensor.polygon, currentSensor.polygon, centerOfRotation)
		previousCollidingEdges = self._getCollidingEdgesByEdge(previousSensor, envMap)
		currentCollidingEdges = self._getCollidingEdgesByEdge(currentSensor, envMap)
		intermediateCollisions = self._findColissionsWithExtendedBb(previousSensor, transformation, centerOfRotation, envMap)
		# Remove the edges that we are sure are intersecting
		for id in previousCollidingEdges:
			for l in previousCollidingEdges[id]:
				if id in intermediateCollisions: intermediateCollisions[id].remove(l)
		for id in currentCollidingEdges:
			for l in currentCollidingEdges[id]:
				if id in intermediateCollisions: intermediateCollisions[id].remove(l)
		collisionIntervals = self._initColissionIntervals(previousSensor, intermediateCollisions)
		# [self.lines.append(e[1]) for e in collisionIntervals]

		(ingoingIntervals, outgoingIntervals) = ([], [])
		while len(collisionIntervals) > 0:
			self._findEventIntervalsForCollisions(previousSensor, collisionIntervals, transformation, centerOfRotation, ingoingIntervals, outgoingIntervals)
		# Each interval is represented as a tuple: (sensorEdge, mapEdge, float, float)
		# The first float is the interval start and the second one is interval end times.
		intervals = self._initEdgeIntervals(previousSensor, transformation, centerOfRotation, previousCollidingEdges, inverse=False)
		haveOverlap = intervals + outgoingIntervals
		dontHaveOverlap = []
		eventCandidates = []
		while len(haveOverlap) > 0:
			i = 0
			while i < len(haveOverlap):
				(sensorEdge, mapEdge, intervalStart, intervalEnd) = haveOverlap.pop(i)
				intervalMid = (intervalStart + intervalEnd) / 2
				collsionCheckResults = self._checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, centerOfRotation, intervalStart, intervalMid)
				if collsionCheckResults[0] != collsionCheckResults[1]:
					haveOverlap.insert(i, (sensorEdge, mapEdge, intervalStart, intervalMid))
					i += 1
				collsionCheckResults = self._checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, centerOfRotation, intervalMid, intervalEnd)
				if collsionCheckResults[0] != collsionCheckResults[1]:
					haveOverlap.insert(i, (sensorEdge, mapEdge, intervalMid, intervalEnd))
					i += 1
			(haveOverlap, dontHaveOverlap) = self._splitIntervalsListForOverlap(haveOverlap)
			for interval in dontHaveOverlap:
				intermediateTransform = Geometry.getParameterizedAffineTransformation(transformation, interval[3])
				p = Geometry.applyMatrixTransformToPolygon(intermediateTransform, previousSensor.polygon, centerOfRotation)
				eventCandidates.append((p, interval[3], "outgoing", interval[1]))
		intervals = self._initEdgeIntervals(currentSensor, transformation, centerOfRotation, currentCollidingEdges, inverse=True)
		haveOverlap = intervals + ingoingIntervals
		dontHaveOverlap = []
		while len(haveOverlap) > 0:
			i = 0
			while i < len(haveOverlap):
				(sensorEdge, mapEdge, intervalStart, intervalEnd) = haveOverlap.pop(i)
				intervalMid = (intervalStart + intervalEnd) / 2
				collsionCheckResults = self._checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, centerOfRotation, intervalStart, intervalMid)
				if collsionCheckResults[0] != collsionCheckResults[1]:
					haveOverlap.insert(i, (sensorEdge, mapEdge, intervalStart, intervalMid))
					i += 1
				collsionCheckResults = self._checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, centerOfRotation, intervalMid, intervalEnd)
				if collsionCheckResults[0] != collsionCheckResults[1]:
					haveOverlap.insert(i, (sensorEdge, mapEdge, intervalMid, intervalEnd))
					i += 1
			(haveOverlap, dontHaveOverlap) = self._splitIntervalsListForOverlap(haveOverlap)
			for interval in dontHaveOverlap:
				intermediateTransform = Geometry.getParameterizedAffineTransformation(transformation, interval[3])
				p = Geometry.applyMatrixTransformToPolygon(intermediateTransform, previousSensor.polygon, centerOfRotation)
				eventCandidates.append((p, interval[3], "ingoing", interval[1]))
		# Sort events by time
		eventCandidates.sort(key=lambda e: e[1])
		# Remove duplicate times
		times = set()
		i = 0
		while i < len(eventCandidates):
			if eventCandidates[i][1] not in times:
				times.add(eventCandidates[i][1])
				i += 1
			else:
				eventCandidates.pop(i)
		# (FOV polygon, timeofEvent, "ingoing" | "outgoing", map edge relevant to the event)
		return eventCandidates

	def _appendConnectivityGraphPerEvent(self, envMap: Map, events: Tuple[Polygon, float, str, LineString], validators: Dict[str, Validator], startTime: float, endTime: float):
		graphs = []
		for event in events:
			time = ((event[1] * (endTime - startTime)) + startTime)
			graph = ConnectivityGraph(envMap, event[0], time, validators)
			graphs.append(graph)
		return graphs

	def _addTemporalEdges(self, eventGraphs: List[ConnectivityGraph], centerOfRotation: Geometry.Coords):
		for graph in eventGraphs:
			isInitialGraph = (len(self.graphs) == 0)
			self._appendGraph(graph)
			if isInitialGraph: return
			previousGraph = self.graphs[-2]
			# Add temporal edges between fovs
			for fovNode in previousGraph.fovNodes:
				fovNodeInShadowTree = self._generateTemporalName(fovNode, previousGraph.time)
				fovNodeInCurrentGraph = self._generateTemporalName(fovNode, graph.time)
				self._addEdge(fovNodeInShadowTree, fovNodeInCurrentGraph, isTemporal=True)
			# Add temporal edges between symbols
			for symNode in previousGraph.symbolNodes:
				symNodeInShadowTree = self._generateTemporalName(symNode, previousGraph.time)
				symNodeInCurrentGraph = self._generateTemporalName(symNode, graph.time)
				self._addEdge(symNodeInShadowTree, symNodeInCurrentGraph, isTemporal=True)
			# Add temporal edges between shadows
			for shadowNodeInPreviousGraph in previousGraph.shadowNodes:
				for shadowNodeInCurrentGraph in graph.shadowNodes:
					previousShadowNodeRegion = previousGraph.nodes[shadowNodeInPreviousGraph]["region"]
					currentShadowNodeRegion = graph.nodes[shadowNodeInCurrentGraph]["region"]
					if previousShadowNodeRegion.polygon.intersects(currentShadowNodeRegion.polygon):
						if self._shadowsAreConnectedTemporally(previousGraph, graph, previousGraph.nodes[shadowNodeInPreviousGraph], graph.nodes[shadowNodeInCurrentGraph], centerOfRotation):
							shadowNodeInShadowTree = self._generateTemporalName(shadowNodeInPreviousGraph, previousGraph.time)
							shadowNodeInCurrentGraph = self._generateTemporalName(shadowNodeInCurrentGraph, graph.time)
							self.nodes[shadowNodeInShadowTree]["toTime"] = graph.time
							self._addEdge(shadowNodeInShadowTree, shadowNodeInCurrentGraph, isTemporal=True)
		return

	def _appendGraph(self, graph: ConnectivityGraph):
		for node in graph.nodes:
			temporalName = self._generateTemporalName(node, graph.time)
			self._addNode(temporalName, graph.time)
			GraphAlgorithms.cloneNodeProps(graph.nodes[node], self.nodes[temporalName])
		for edge in graph.edges:
			frm = self._generateTemporalName(edge[0], graph.time)
			to = self._generateTemporalName(edge[1], graph.time)
			self._addEdge(frm, to, False)
		self.graphs.append(graph)
		return

	def _build(self, envMap: Map, fovs: List[Fov], validators: Dict[str, Validator], startInd = 0, endInd = None):
		if endInd is None: endInd = len(fovs)

		previousFov: Fov = None
		currentConnectivityG: ConnectivityGraph = None
		startTime = time.time()

		for i in range(startInd, endInd):
			currentFov = fovs[i]
			if previousFov is None:
				currentConnectivityG = ConnectivityGraph(envMap, currentFov.polygon, currentFov.time, validators)
				self._appendGraph(currentConnectivityG)
				previousFov = currentFov
			else:
				for sensorId in previousFov.sensors:
					previousSensor = previousFov.sensors[sensorId]
					currentSensor = currentFov.getEquivalentSensorById(sensorId)
					centerOfRotation = (previousSensor.pose.x, previousSensor.pose.y)
					componentEvents = self._findIntermediateComponentEvents(previousSensor.region, currentSensor.region, centerOfRotation, envMap)
					self.componentEvents.append(componentEvents)
					graphs = self._appendConnectivityGraphPerEvent(envMap, componentEvents, validators, previousFov.time, currentFov.time)
					self._addTemporalEdges(graphs, centerOfRotation)
				previousFov = currentFov
		print("took %.2fms" % (time.time() - startTime))
		return

	def displayGraph(self, displayGeomGraph, displaySpringGraph):
		self._fig = GraphAlgorithms.displayGraphAuto(self, displayGeomGraph, displaySpringGraph)

	def killDisplayedGraph(self):
		if self._fig:
			GraphAlgorithms.killDisplayedGraph(self._fig)
			self._fig = None
