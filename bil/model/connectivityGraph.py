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

	def condense(self):
		return # NOOP

	def _addNode(self, nodeName):
		self.add_node(nodeName)
		self.nodes[nodeName]["timestamp"] = self.timestamp

	def _addEdges(self, fromStr, toStr):
		self.add_edge(fromStr, toStr)
		self.add_edge(toStr, fromStr)

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
