import re
import networkx as nx
from shapely.geometry import LineString, Point, Polygon, MultiPolygon
from typing import List, Set, Dict, Union

from bil.model.polygonalRegion import PolygonalRegion
from bil.model.sensingRegion import SensingRegion
from bil.model.shadowRegion import ShadowRegion
from bil.model.map import Map
from bil.observation.fov import FOV
from bil.utils.geometry import Geometry
from bil.utils.graph import GraphAlgorithms

class NodeCluster:
	def __init__(self, cGraph: "ConnectivityGraph", initialNode: str):
		self._name = initialNode
		self._cGraph = cGraph
		self.nodes: Set[str] = { initialNode }
		self._polygon = None

	def __repr__(self) -> str:
		return "NodeCluster-%s" % self._name

	@property
	def polygon(self):
		if self._polygon is None:
			self._polygon = Geometry.union([self._cGraph.nodes[nodeName]["region"].polygon for nodeName in self.nodes])
		return self._polygon

class ConnectivityGraph(nx.DiGraph):
	def __init__(self, envMap: Map, fov: FOV, validators):
		super().__init__()
		self.fov = fov
		self.validators = validators
		self.shadows: List[str] = []
		self.fovComponents: List[str] = []
		self.map: Map = envMap
		self.timestamp = fov.time
		self._disjointPolys: List[PolygonalRegion] = []
		print("Building graph for %s" % self.timestamp)
		self._build(fov)
		self._fig = None
		self._condensed = None
		self.nodeClusters: Dict[str, NodeCluster] = {}
		self.nodeToClusterMap: Dict[str, str] = {}
		self._OBSOLETE_nodeClusters: Dict[str, Set[str]] = {}
		self._OBSOLETE_nodeToClusterMap: Dict[str, str] = {}
		self._OBSOLETE_condensed = None

	def __repr__(self):
		return "cGraph-%.2f" % self.timestamp

	def _addCluster(self, node):
		self.nodeClusters[node] = NodeCluster(self, node)
		self.nodeToClusterMap[node] = node

	def _condenseBeam(self, beam):
		(start, end) = GraphAlgorithms.getBeamEnds(beam)
		if start in self.nodeToClusterMap:
			start = self.nodeToClusterMap[start]
		if end in self.nodeToClusterMap:
			end = self.nodeToClusterMap[end]
		return GraphAlgorithms.getBeamName(start, end)

	def condense(self):
		if self._condensed is not None: return self._condensed
		print("Condensing %s" % self.timestamp)
		self.nodeClusters: Dict[str, NodeCluster] = {}
		self.nodeToClusterMap: Dict[str, str] = {}
		for n1 in self.nodes:
			if n1 in self.nodeToClusterMap: continue
			if GraphAlgorithms.isBeamNode(n1):
				self._addCluster(n1)
				continue
			if n1 not in self.nodeToClusterMap:
				self._addCluster(n1)

			for n2 in self.nodes:
				if n2 in self.nodeToClusterMap: continue
				if n1 == n2: continue
				if GraphAlgorithms.isBeamNode(n2):
					self._addCluster(n2)
					continue
				# FIXME: This should happen in one DFS not n^2
				if GraphAlgorithms.isConnectedBfs(self, n1, n2):
					if n1 in self.nodeToClusterMap:
						self.nodeClusters[self.nodeToClusterMap[n1]].nodes.add(n2)
						self.nodeToClusterMap[n2] = self.nodeToClusterMap[n1]
					elif n2 in self.nodeToClusterMap:
						self.nodeClusters[self.nodeToClusterMap[n2]].nodes.add(n1)
						self.nodeToClusterMap[n1] = self.nodeToClusterMap[n2]
					else:
						self._addCluster(n1)
						self.nodeClusters[self.nodeToClusterMap[n1]].nodes.add(n2)
						self.nodeToClusterMap[n2] = self.nodeToClusterMap[n1]

		# We found the nodes
		# Now add edges by looking at the original graph
		condensedGraph = nx.DiGraph()
		condensedGraph.timestamp = self.timestamp
		for n in self.nodeClusters:
			newNodeName = self._condenseBeam(n) if GraphAlgorithms.isBeamNode(n) else n
			self.nodeToClusterMap[newNodeName] = n
			condensedGraph.add_node(newNodeName)
			condensedGraph.nodes[newNodeName]["mappedName"] = n
			GraphAlgorithms.cloneNodeProps(self.nodes[n], condensedGraph.nodes[newNodeName])
		for n1 in self.nodes:
			for n2 in self.adj[n1]:
				n1Cluster = self.nodeToClusterMap[n1]
				n2Cluster = self.nodeToClusterMap[n2]
				if GraphAlgorithms.isBeamNode(n1Cluster):
					n1Cluster = self._condenseBeam(n1Cluster)
				if GraphAlgorithms.isBeamNode(n2Cluster):
					n2Cluster = self._condenseBeam(n2Cluster)
				condensedGraph.add_edge(n1Cluster, n2Cluster)

		# So far we have all the FOV and shadows condensed
		# Now we need to put the Spec polygons into the Graph
		# for validatorName in self.validators:
		# 	validator = self.validators[validatorName]
		# 	if validator.isRegion:
		# 		region1 = validator.value
		# 		condensedGraph.add_node(region1.name)
		# 		condensedGraph.nodes[region1.name]["timestamp"] = self.timestamp
		# 		condensedGraph.nodes[region1.name]["region"] = region1
		# 		condensedGraph.nodes[region1.name]["centroid"] = region1.polygon.centroid
		# 		condensedGraph.nodes[region1.name]["mappedName"] = region1.name
		# 		self._addCluster(region1.name)
		# 		for n in self.nodeToClusterMap:
		# 			if GraphAlgorithms.isBeamNode(n): continue
		# 			region2 = self.nodes[n]["region"]
		# 			if Geometry.polygonAndPolygonIntersect(region1.polygon, region2.polygon):
		# 				condensedGraph.nodes[region1.name]["type"] = "shadow" if isinstance(region2, ShadowRegion) else "sensor"
		# 				condensedGraph.add_edge(region1.name, self.nodeToClusterMap[n])
		# 				break

		print(len(self.nodes))
		print(len(condensedGraph.nodes))
		self._condensed = condensedGraph
		return self._condensed

	def _addBeamNodes(self, rOut, rIn):
		out2in = GraphAlgorithms.getBeamName(rOut, rIn)
		in2out = GraphAlgorithms.getBeamName(rIn, rOut)
		self._addBeam(out2in)
		self._addBeam(in2out)
		self.add_edge(rOut, out2in)
		self.add_edge(out2in, rIn)
		self.add_edge(rIn, in2out)
		self.add_edge(in2out, rOut)

	def _addNode(self, nodeName):
		self.add_node(nodeName)
		self.nodes[nodeName]["timestamp"] = self.timestamp

	def _addEdges(self, fromStr, toStr):
		self.add_edge(fromStr, toStr)
		self.add_edge(toStr, fromStr)

	def _addRegion(self, polygonalRegion):
		name = polygonalRegion.name
		self._addNode(name)
		self.nodes[name]["region"] = polygonalRegion
		self.nodes[name]["centroid"] = polygonalRegion.polygon.centroid
		self.nodes[name]["type"] = "shadow" if isinstance(polygonalRegion, ShadowRegion) else "sensor"
		self._regionNodes.append(name)

	def _addBeam(self, name):
		self._addNode(name)
		self._beamNodes.append(name)

	def _getMapRegion(self, name):
		return self.map.regions.get(name.split("-")[0], None)

	def _addSingleSensorRegionConnectedComponent(self, connectedComponent, index):
		name = "FOV-%d" % index
		region = SensingRegion(name, [], self.timestamp, index, polygon=connectedComponent)
		self.fovComponents.append(name)
		self._addNode(name)
		self.nodes[name]["region"] = region
		self.nodes[name]["centroid"] = region.polygon.centroid
		self.nodes[name]["type"] = "sensor"
		return

	def _addSensorRegionConnectedComponents(self, fovUnion: Union[Polygon, MultiPolygon]):
		if isinstance(fovUnion, MultiPolygon):
			i = 0
			for connectedComponent in fovUnion:
				self._addSingleSensorRegionConnectedComponent(connectedComponent, i)
				i += 1
		else:
			self._addSingleSensorRegionConnectedComponent(fovUnion, 0)
		return

	def _createNewShadow(self, shadow: Polygon):
		name = "S-%d" % len(self.shadows)
		region = ShadowRegion(name, Geometry.getPolygonCoords(shadow))
		self.shadows.append(name)
		self._addNode(name)
		self.nodes[name]["region"] = region
		self.nodes[name]["centroid"] = region.polygon.centroid
		self.nodes[name]["type"] = "shadow"
		for fovName in self.fovComponents:
			fovComponent = self.nodes[fovName]["region"]
			if Geometry.haveOverlappingEdge(fovComponent.polygon, region.polygon):
				self._addEdges(fovName, name)
		return

	def _mergeShadow(self, existingShadow: ShadowRegion, shadow: Polygon):
		p = Geometry.union([existingShadow.polygon, shadow])
		region = ShadowRegion(existingShadow.name, Geometry.getPolygonCoords(p))
		self.nodes[existingShadow.name]["region"] = region
		self.nodes[existingShadow.name]["centroid"] = region.polygon.centroid
		return

	def _updateShadows(self, newShadow: Polygon):
		if len(self.shadows) == 0:
			self._createNewShadow(newShadow)
		else:
			beforeUpdate = self.shadows.copy()
			for existingShadowName in beforeUpdate:
				existingShadow = self.nodes[existingShadowName]["region"]
				if Geometry.haveOverlappingEdge(existingShadow.polygon, newShadow):
					self._mergeShadow(existingShadow, newShadow)
				else:
					self._createNewShadow(newShadow)
		return

	def _constructShadows(self, fovUnion: Union[Polygon, MultiPolygon]):
		allShadowPolygons = []
		for regionName in self.map.regions:
			mapR = self.map.regions[regionName]
			if Geometry.polygonAndPolygonIntersect(mapR.polygon, fovUnion):
				shadows = Geometry.shadows(mapR.polygon, fovUnion)
				shadows = [p for p in shadows if p.length > 0]
				shadows = list(filter(lambda p: not isinstance(p, LineString), shadows))
				allShadowPolygons = allShadowPolygons + shadows
			else:
				allShadowPolygons.append(mapR.polygon)
		mergedPolys = Geometry.union(allShadowPolygons)
		for shadow in mergedPolys:
			self._updateShadows(shadow)
		return

	def _isValidatorPolygon(self, polygonName):
		"""
		Utility function to check if this polygon is made for a validator region
		"""
		return polygonName.startswith("sym-")

	def _build(self, fov):
		# First we create all the nodes with respect to FOV
		fovUnion = fov.polygon
		self._addSensorRegionConnectedComponents(fovUnion)
		self._constructShadows(fovUnion)
		return

	def getSensorReadings(self, startPose, endPose):
		startPolygonName = None
		endPolygonName = None
		for region in self._disjointPolys:
			if Geometry.isXyInsidePolygon(startPose.x, startPose.y, region.polygon):
				startPolygonName = region.name
			if Geometry.isXyInsidePolygon(endPose.x, endPose.y, region.polygon):
				endPolygonName = region.name
			if startPolygonName is not None and endPolygonName is not None:
				break
			beam = None
			if self._OBSOLETE_nodeToClusterMap[startPolygonName] != self._OBSOLETE_nodeToClusterMap[endPolygonName]:
				beam = GraphAlgorithms.getBeamName(startPolygonName, endPolygonName)
				beam = self._OBSOLETE_condenseBeam(beam)
		return [self._OBSOLETE_nodeToClusterMap[startPolygonName], beam ] if beam is not None else [self._OBSOLETE_nodeToClusterMap[startPolygonName]]

	def _findBeamNode(self, l: LineString, p: PolygonalRegion):
		for other in self._disjointPolys:
			if p == other: continue
			common = p.getCommonEdge(other)
			if not common: continue
			pt = Geometry.intersectLineSegments(common, l)
			if not pt: continue
			(dx, dy) = Geometry.getDirectionXyFromLineSeg(l)
			pt = Geometry.pushPointEpsilon(dx, dy, pt)
			if Geometry.isPointInsidePolygon(pt, p.polygon):
				return GraphAlgorithms.getBeamName(other.name, p.name)
			return GraphAlgorithms.getBeamName(p.name, other.name)
		raise RuntimeError("This shouldn't ever happen")

	def displayGraph(self, displayGeomGraph, displaySpringGraph):
		if displayGeomGraph:
			return
		else:
			self._fig = GraphAlgorithms.displaySpringGraph(self, self.fovComponents, self.shadows)
		return

	def killDisplayedGraph(self):
		if self._fig:
			GraphAlgorithms.killDisplayedGraph(self._fig)
			self._fig = None

	###### OBSOLETE CODE

	def _OBSOLETE_condense(self, spec):
		if self._OBSOLETE_condensed is not None: return self._OBSOLETE_condensed
		print("Condensing %s" % self.timestamp)
		self._OBSOLETE_nodeClusters: Dict[str, Set[str]] = {}
		self._OBSOLETE_nodeToClusterMap: Dict[str, str] = {}
		self._OBSOLETE_condensed = self._OBSOLETE_condense(spec)
		return self._OBSOLETE_condensed

	def _OBSOLETE_addCluster(self, node):
		self._OBSOLETE_nodeClusters[node] = { node }
		self._OBSOLETE_nodeToClusterMap[node] = node

	def _OBSOLETE_condenseBeam(self, beam):
		(start, end) = GraphAlgorithms.getBeamEnds(beam)
		if start in self._OBSOLETE_nodeToClusterMap:
			start = self._OBSOLETE_nodeToClusterMap[start]
		if end in self._OBSOLETE_nodeToClusterMap:
			end = self._OBSOLETE_nodeToClusterMap[end]
		return GraphAlgorithms.getBeamName(start, end)

	def _OBSOLETE_condense(self, spec) -> nx.DiGraph:
		for n1 in self.nodes:
			if n1 in self._OBSOLETE_nodeToClusterMap: continue
			if GraphAlgorithms.isBeamNode(n1):
				self._OBSOLETE_addCluster(n1)
				continue
			if n1 not in self._OBSOLETE_nodeToClusterMap:
				self._OBSOLETE_addCluster(n1)
			else:
				r1 = self._getMapRegion(n1)
				if r1 is not None and r1.name in spec:
					self._OBSOLETE_addCluster(n1)

			for n2 in self.nodes:
				if n2 in self._OBSOLETE_nodeToClusterMap: continue
				if n1 == n2: continue
				if GraphAlgorithms.isBeamNode(n2):
					self._OBSOLETE_addCluster(n2)
					continue
				r2 = self._getMapRegion(n2)
				if r2 is not None and r2.name in spec:
					self._OBSOLETE_addCluster(n2)
					continue
				# FIXME: This should happen in one DFS not n^2
				if GraphAlgorithms.isConnectedBfs(self, n1, n2):
					if n1 in self._OBSOLETE_nodeToClusterMap:
						self._OBSOLETE_nodeClusters[self._OBSOLETE_nodeToClusterMap[n1]].add(n2)
						self._OBSOLETE_nodeToClusterMap[n2] = self._OBSOLETE_nodeToClusterMap[n1]
					elif n2 in self._OBSOLETE_nodeToClusterMap:
						self._OBSOLETE_nodeClusters[self._OBSOLETE_nodeToClusterMap[n2]].add(n1)
						self._OBSOLETE_nodeToClusterMap[n1] = self._OBSOLETE_nodeToClusterMap[n2]
					else:
						self._OBSOLETE_addCluster(n1)
						self._OBSOLETE_nodeClusters[self._OBSOLETE_nodeToClusterMap[n1]].add(n2)
						self._OBSOLETE_nodeToClusterMap[n2] = self._OBSOLETE_nodeToClusterMap[n1]

		# We found the nodes
		# Now add edges by looking at the original graph
		condensedGraph = nx.DiGraph()
		condensedGraph.timestamp = self.timestamp
		for n in self._OBSOLETE_nodeClusters:
			newNodeName = self._OBSOLETE_condenseBeam(n) if GraphAlgorithms.isBeamNode(n) else n
			self._OBSOLETE_nodeToClusterMap[newNodeName] = n
			condensedGraph.add_node(newNodeName)
			condensedGraph.nodes[newNodeName]["mappedName"] = n
			GraphAlgorithms.cloneNodeProps(self.nodes[n], condensedGraph.nodes[newNodeName])
		for n1 in self.nodes:
			for n2 in self.adj[n1]:
				n1Cluster = self._OBSOLETE_nodeToClusterMap[n1]
				n2Cluster = self._OBSOLETE_nodeToClusterMap[n2]
				if GraphAlgorithms.isBeamNode(n1Cluster):
					n1Cluster = self._OBSOLETE_condenseBeam(n1Cluster)
				if GraphAlgorithms.isBeamNode(n2Cluster):
					n2Cluster = self._OBSOLETE_condenseBeam(n2Cluster)
				condensedGraph.add_edge(n1Cluster, n2Cluster)
		print(len(self.nodes))
		print(len(condensedGraph.nodes))
		return condensedGraph
