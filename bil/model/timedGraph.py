from shapely import affinity
from shapely.geometry import LineString, Point, Polygon
from typing import List, Set, Dict
import networkx as nx
import time

from bil.model.connectivityGraph import ConnectivityGraph
from bil.utils.geometry import Geometry
from bil.utils.graph import GraphAlgorithms
from bil.model.map import Map

class TimedGraph(nx.DiGraph):
	def __init__(self, graphs: List[ConnectivityGraph], startInd = 0, endInd = None):
		super().__init__()
		self._nodeLayers = []
		self._timeStamps = []
		self._regionNodes = []
		self._beamNodes = []
		self._beamSet = set()
		self._boundary = None
		self.reza = []
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

	def _buildFovCoordsMap(self, currentLayer: ConnectivityGraph, previousLayer: ConnectivityGraph) -> Geometry.CoordsMap:
		"""
		Map from point in bottom layer to top layer
		"""
		fovCoordsMap: List[Geometry.CoordsMap] = []
		for sensorKey in currentLayer.fov.sensors:
			currentLayerDict = {}
			fovCoordsMap.append(currentLayerDict)
			currentSensor = currentLayer.fov.sensors[sensorKey]
			previousKey = (previousLayer.fov.time, sensorKey[1])
			previousSensor = previousLayer.fov.sensors[previousKey]
			for i in range(len(previousSensor._originalCoords)):
				currentLayerDict[currentSensor._originalCoords[i]] = previousSensor._originalCoords[i]
		return fovCoordsMap

	def _testConnectivity(self, x1: float, y1: float, x2: float, y2: float, x3: float, y3: float, x4: float, y4: float, px: float, py: float):
		"""
		In oneNote look at 03/02/2021
		syms l x1 x2 x3 x4 y1 y2 y3 y4 a b xx yy xxx yyy
		eqns = [
		l*x3+(1-l)*x1==xx,
		l*y3+(1-l)*y1 == yy,
		l*x4+(1-l)*x2 == xxx,
		l*y4+(1-l)*y2 == yyy,
		a*xx+b==yy,
		a*xxx+b==yyy,
		a*px+b==py
		]
		solve(eqns, [l,xx,yy,xxx,yyy,a,b])

		ans.l =
		(px^2*y1 - px^2*y2 - px^2*y3 + px^2*y4 + px*(px^2*y1^2 - 2*px^2*y1*y2 - 2*px^2*y1*y3 + 2*px^2*y1*y4 + px^2*y2^2 + 2*px^2*y2*y3 - 2*px^2*y2*y4 + px^2*y3^2 - 2*px^2*y3*y4 + px^2*y4^2 - 2*px*py*x1*y1 + 2*px*py*x1*y2 + 2*px*py*x1*y3 - 2*px*py*x1*y4 + 2*px*py*x2*y1 - 2*px*py*x2*y2 - 2*px*py*x2*y3 + 2*px*py*x2*y4 + 2*px*py*x3*y1 - 2*px*py*x3*y2 - 2*px*py*x3*y3 + 2*px*py*x3*y4 - 2*px*py*x4*y1 + 2*px*py*x4*y2 + 2*px*py*x4*y3 - 2*px*py*x4*y4 + 2*px*x1*y1*y4 - 4*px*x1*y2*y3 + 2*px*x1*y2*y4 + 2*px*x1*y3*y4 - 2*px*x1*y4^2 + 2*px*x2*y1*y3 - 4*px*x2*y1*y4 + 2*px*x2*y2*y3 - 2*px*x2*y3^2 + 2*px*x2*y3*y4 + 2*px*x3*y1*y2 - 4*px*x3*y1*y4 - 2*px*x3*y2^2 + 2*px*x3*y2*y3 + 2*px*x3*y2*y4 - 2*px*x4*y1^2 + 2*px*x4*y1*y2 + 2*px*x4*y1*y3 + 2*px*x4*y1*y4 - 4*px*x4*y2*y3 + py^2*x1^2 - 2*py^2*x1*x2 - 2*py^2*x1*x3 + 2*py^2*x1*x4 + py^2*x2^2 + 2*py^2*x2*x3 - 2*py^2*x2*x4 + py^2*x3^2 - 2*py^2*x3*x4 + py^2*x4^2 - 2*py*x1^2*y4 + 2*py*x1*x2*y3 + 2*py*x1*x2*y4 + 2*py*x1*x3*y2 + 2*py*x1*x3*y4 + 2*py*x1*x4*y1 - 4*py*x1*x4*y2 - 4*py*x1*x4*y3 + 2*py*x1*x4*y4 - 2*py*x2^2*y3 - 4*py*x2*x3*y1 + 2*py*x2*x3*y2 + 2*py*x2*x3*y3 - 4*py*x2*x3*y4 + 2*py*x2*x4*y1 + 2*py*x2*x4*y3 - 2*py*x3^2*y2 + 2*py*x3*x4*y1 + 2*py*x3*x4*y2 - 2*py*x4^2*y1 + x1^2*y4^2 - 2*x1*x2*y3*y4 - 2*x1*x3*y2*y4 - 2*x1*x4*y1*y4 + 4*x1*x4*y2*y3 + x2^2*y3^2 + 4*x2*x3*y1*y4 - 2*x2*x3*y2*y3 - 2*x2*x4*y1*y3 + x3^2*y2^2 - 2*x3*x4*y1*y2 + x4^2*y1^2)^(1/2) - px*py*x1 + px*py*x2 + px*py*x3 - px*py*x4 + 2*py*x1*x4 - 2*py*x2*x3 - px*x1*y4 + px*x2*y3 + px*x3*y2 - px*x4*y1)/(2*(px*x1*y2 - px*x2*y1 - px*x1*y4 + px*x2*y3 - px*x3*y2 + px*x4*y1 + px*x3*y4 - px*x4*y3)) - (py*x1*x4 - py*x2*x3 - px*x1*y2 + px*x2*y1 + px*x3*y2 - px*x4*y1)/(px*x1*y2 - px*x2*y1 - px*x1*y4 + px*x2*y3 - px*x3*y2 + px*x4*y1 + px*x3*y4 - px*x4*y3)
		- (py*x1*x4 - py*x2*x3 - px*x1*y2 + px*x2*y1 + px*x3*y2 - px*x4*y1)/(px*x1*y2 - px*x2*y1 - px*x1*y4 + px*x2*y3 - px*x3*y2 + px*x4*y1 + px*x3*y4 - px*x4*y3) - (px^2*y2 - px^2*y1 + px^2*y3 - px^2*y4 + px*(px^2*y1^2 - 2*px^2*y1*y2 - 2*px^2*y1*y3 + 2*px^2*y1*y4 + px^2*y2^2 + 2*px^2*y2*y3 - 2*px^2*y2*y4 + px^2*y3^2 - 2*px^2*y3*y4 + px^2*y4^2 - 2*px*py*x1*y1 + 2*px*py*x1*y2 + 2*px*py*x1*y3 - 2*px*py*x1*y4 + 2*px*py*x2*y1 - 2*px*py*x2*y2 - 2*px*py*x2*y3 + 2*px*py*x2*y4 + 2*px*py*x3*y1 - 2*px*py*x3*y2 - 2*px*py*x3*y3 + 2*px*py*x3*y4 - 2*px*py*x4*y1 + 2*px*py*x4*y2 + 2*px*py*x4*y3 - 2*px*py*x4*y4 + 2*px*x1*y1*y4 - 4*px*x1*y2*y3 + 2*px*x1*y2*y4 + 2*px*x1*y3*y4 - 2*px*x1*y4^2 + 2*px*x2*y1*y3 - 4*px*x2*y1*y4 + 2*px*x2*y2*y3 - 2*px*x2*y3^2 + 2*px*x2*y3*y4 + 2*px*x3*y1*y2 - 4*px*x3*y1*y4 - 2*px*x3*y2^2 + 2*px*x3*y2*y3 + 2*px*x3*y2*y4 - 2*px*x4*y1^2 + 2*px*x4*y1*y2 + 2*px*x4*y1*y3 + 2*px*x4*y1*y4 - 4*px*x4*y2*y3 + py^2*x1^2 - 2*py^2*x1*x2 - 2*py^2*x1*x3 + 2*py^2*x1*x4 + py^2*x2^2 + 2*py^2*x2*x3 - 2*py^2*x2*x4 + py^2*x3^2 - 2*py^2*x3*x4 + py^2*x4^2 - 2*py*x1^2*y4 + 2*py*x1*x2*y3 + 2*py*x1*x2*y4 + 2*py*x1*x3*y2 + 2*py*x1*x3*y4 + 2*py*x1*x4*y1 - 4*py*x1*x4*y2 - 4*py*x1*x4*y3 + 2*py*x1*x4*y4 - 2*py*x2^2*y3 - 4*py*x2*x3*y1 + 2*py*x2*x3*y2 + 2*py*x2*x3*y3 - 4*py*x2*x3*y4 + 2*py*x2*x4*y1 + 2*py*x2*x4*y3 - 2*py*x3^2*y2 + 2*py*x3*x4*y1 + 2*py*x3*x4*y2 - 2*py*x4^2*y1 + x1^2*y4^2 - 2*x1*x2*y3*y4 - 2*x1*x3*y2*y4 - 2*x1*x4*y1*y4 + 4*x1*x4*y2*y3 + x2^2*y3^2 + 4*x2*x3*y1*y4 - 2*x2*x3*y2*y3 - 2*x2*x4*y1*y3 + x3^2*y2^2 - 2*x3*x4*y1*y2 + x4^2*y1^2)^(1/2) + px*py*x1 - px*py*x2 - px*py*x3 + px*py*x4 - 2*py*x1*x4 + 2*py*x2*x3 + px*x1*y4 - px*x2*y3 - px*x3*y2 + px*x4*y1)/(2*(px*x1*y2 - px*x2*y1 - px*x1*y4 + px*x2*y3 - px*x3*y2 + px*x4*y1 + px*x3*y4 - px*x4*y3))
		"""
		bigParenthesis = px**2*y1**2 - 2*px**2*y1*y2 - 2*px**2*y1*y3 + 2*px**2*y1*y4 + px**2*y2**2 + 2*px**2*y2*y3 - 2*px**2*y2*y4 + px**2*y3**2 - 2*px**2*y3*y4 + px**2*y4**2 - 2*px*py*x1*y1 + 2*px*py*x1*y2 + 2*px*py*x1*y3 - 2*px*py*x1*y4 + 2*px*py*x2*y1 - 2*px*py*x2*y2 - 2*px*py*x2*y3 + 2*px*py*x2*y4 + 2*px*py*x3*y1 - 2*px*py*x3*y2 - 2*px*py*x3*y3 + 2*px*py*x3*y4 - 2*px*py*x4*y1 + 2*px*py*x4*y2 + 2*px*py*x4*y3 - 2*px*py*x4*y4 + 2*px*x1*y1*y4 - 4*px*x1*y2*y3 + 2*px*x1*y2*y4 + 2*px*x1*y3*y4 - 2*px*x1*y4**2 + 2*px*x2*y1*y3 - 4*px*x2*y1*y4 + 2*px*x2*y2*y3 - 2*px*x2*y3**2 + 2*px*x2*y3*y4 + 2*px*x3*y1*y2 - 4*px*x3*y1*y4 - 2*px*x3*y2**2 + 2*px*x3*y2*y3 + 2*px*x3*y2*y4 - 2*px*x4*y1**2 + 2*px*x4*y1*y2 + 2*px*x4*y1*y3 + 2*px*x4*y1*y4 - 4*px*x4*y2*y3 + py**2*x1**2 - 2*py**2*x1*x2 - 2*py**2*x1*x3 + 2*py**2*x1*x4 + py**2*x2**2 + 2*py**2*x2*x3 - 2*py**2*x2*x4 + py**2*x3**2 - 2*py**2*x3*x4 + py**2*x4**2 - 2*py*x1**2*y4 + 2*py*x1*x2*y3 + 2*py*x1*x2*y4 + 2*py*x1*x3*y2 + 2*py*x1*x3*y4 + 2*py*x1*x4*y1 - 4*py*x1*x4*y2 - 4*py*x1*x4*y3 + 2*py*x1*x4*y4 - 2*py*x2**2*y3 - 4*py*x2*x3*y1 + 2*py*x2*x3*y2 + 2*py*x2*x3*y3 - 4*py*x2*x3*y4 + 2*py*x2*x4*y1 + 2*py*x2*x4*y3 - 2*py*x3**2*y2 + 2*py*x3*x4*y1 + 2*py*x3*x4*y2 - 2*py*x4**2*y1 + x1**2*y4**2 - 2*x1*x2*y3*y4 - 2*x1*x3*y2*y4 - 2*x1*x4*y1*y4 + 4*x1*x4*y2*y3 + x2**2*y3**2 + 4*x2*x3*y1*y4 - 2*x2*x3*y2*y3 - 2*x2*x4*y1*y3 + x3**2*y2**2 - 2*x3*x4*y1*y2 + x4**2*y1**2
		smallParenthesis1 = py*x1*x4 - py*x2*x3 - px*x1*y2 + px*x2*y1 + px*x3*y2 - px*x4*y1
		smallParenthesis2 = px*x1*y2 - px*x2*y1 - px*x1*y4 + px*x2*y3 - px*x3*y2 + px*x4*y1 + px*x3*y4 - px*x4*y3
		l1 = (px**2*y1 - px**2*y2 - px**2*y3 + px**2*y4 + px*(bigParenthesis)**(1/2) - px*py*x1 + px*py*x2 + px*py*x3 - px*py*x4 + 2*py*x1*x4 - 2*py*x2*x3 - px*x1*y4 + px*x2*y3 + px*x3*y2 - px*x4*y1)/(2*(smallParenthesis2)) - (smallParenthesis1)/(smallParenthesis2)
		l2 = -1 * (smallParenthesis1)/(smallParenthesis2) - (px**2*y2 - px**2*y1 + px**2*y3 - px**2*y4 + px*(bigParenthesis)**(1/2) + px*py*x1 - px*py*x2 - px*py*x3 + px*py*x4 - 2*py*x1*x4 + 2*py*x2*x3 + px*x1*y4 - px*x2*y3 - px*x3*y2 + px*x4*y1)/(2*(smallParenthesis2))
		l = l1 if l2 < 0 else l2
		if l >= 1:
			return False
		xx = l*x3 + (1 - l)*x1
		yy = l*y3 + (1 - l)*y1
		xxx = l*x4 + (1 - l)*x2
		yyy = l*y4 + (1 - l)*y2
		(xMin, xMax) = (xx, xxx) if xx < xxx else (xxx, xx)
		(yMin, yMax) = (yy, yyy) if yy < yyy else (yyy, yy)
		if px > xMax or px < xMin:
			return False
		if py > yMax or py < yMin:
			return False
		return True

	def _checkChangesInCollidingEdges(self, previousFovPolygon: Polygon, currentFovPolygon: Polygon, envMap: Map) -> Polygon:
		"""
		Takes two connectivity graphs and looks at their respective FOVs
		to see if the edges of the FOV that collides crosses a different set of edges of the map's boundary.
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
		thereIsChange = prevHash != currHash
		return thereIsChange

	def _getAffineTransformation(self, previousFovPolygon: Polygon, currentFovPolygon: Polygon):
		matrix = Geometry.getAffineTransformation(previousFovPolygon.exterior.coords, currentFovPolygon.exterior.coords)
		return matrix

	def _applyAffineTransformation(self, matrix , previousFovPolygon: Polygon):
		transformedPolygon = Geometry.applyMatrixTransformToPolygon(matrix, previousFovPolygon)
		return transformedPolygon

	def _findIntermediateComponentEvents(self, previousFovPolygon: Polygon, currentFovPolygon: Polygon, envMap: Map, depth=0):
		if depth > 5: return
		# Look for changes in colliding edges
		thereIsChange = self._checkChangesInCollidingEdges(previousFovPolygon, currentFovPolygon, envMap)
		# If there are no changes, we don't need to include the layer in our timesteps
		if not thereIsChange: return
		# Find affine transformation, we use the matrix to find all the intermediate component events in this period
		matrix = self._getAffineTransformation(previousFovPolygon, currentFovPolygon)
		newMatrix = Geometry.getParameterizedAffineTransformation(matrix, 0.5)
		intermediatePolygon = self._applyAffineTransformation(newMatrix, previousFovPolygon)
		self.reza.append(intermediatePolygon)
		self._findIntermediateComponentEvents(previousFovPolygon, intermediatePolygon, envMap, depth + 1)
		self._findIntermediateComponentEvents(intermediatePolygon, currentFovPolygon, envMap, depth + 1)

	def _build(self, graphs: List[ConnectivityGraph], startInd = 0, endInd = None):
		if endInd is None: endInd = len(graphs)

		self.nodesByLayer = []
		previousLayer = None
		startTime = time.time()
		for i in range(startInd, endInd):
			currentLayer = graphs[i]
			if previousLayer is not None:
				self._findIntermediateComponentEvents(previousLayer.fov.polygon, currentLayer.fov.polygon, previousLayer.map)
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
					# FIXME: 2- GO TO FIXME: 1 BEFORE THIS - Now do the test for the coordinates to see if any FOV crossed it
					# self._oldAlg(n1 ,n2)
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
