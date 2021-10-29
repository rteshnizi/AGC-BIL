import networkx as nx
from shapely.geometry import LineString, Polygon, MultiPolygon
from typing import Dict, List, Union

from bil.model.polygonalRegion import PolygonalRegion
from bil.model.sensingRegion import SensingRegion
from bil.model.shadowRegion import ShadowRegion
from bil.model.map import Map
from bil.model.validatorRegion import ValidatorRegion
from bil.observation.fov import Fov
from bil.spec.validator import Validator
from bil.utils.geometry import Geometry
from bil.utils.graph import GraphAlgorithms

class ConnectivityGraph(nx.DiGraph):
	def __init__(self, envMap: Map, fovUnion: Union[Polygon, MultiPolygon], timestamp: float, validators: Dict[str, Validator]):
		super().__init__()
		self.validators = validators
		self.fovNodes: List[str] = []
		self.shadowNodes: List[str] = []
		self.symbolNodes: List[str] = []
		self.map: Map = envMap
		self.timestamp = timestamp
		print("Building graph for %s" % self.timestamp)
		self._build(fovUnion)
		self._fig = None

	def __repr__(self):
		return "cGraph-%.2f" % self.timestamp

	def _addNode(self, nodeName: str, region: PolygonalRegion, type: str):
		self.add_node(nodeName)
		self.nodes[nodeName]["region"] = region
		self.nodes[nodeName]["centroid"] = region.polygon.centroid
		self.nodes[nodeName]["type"] = type
		self.nodes[nodeName]["timestamp"] = self.timestamp
		if type == "sensor":
			self.fovNodes.append(nodeName)
		elif type == "shadow":
			self.shadowNodes.append(nodeName)
		elif type == "symbol":
			self.symbolNodes.append(nodeName)
		else:
			raise "Unknown node type %s" % type
		return

	def _addEdges(self, fromStr, toStr):
		self.add_edge(fromStr, toStr)
		self.add_edge(toStr, fromStr)

	def _addSingleSensorRegionConnectedComponent(self, connectedComponent, index):
		name = "FOV-%d" % index
		region = SensingRegion(name, [], self.timestamp, index, polygon=connectedComponent)
		self._addNode(name, region, "sensor")
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
		name = "S-%d" % len(self.shadowNodes)
		region = ShadowRegion(name, Geometry.getPolygonCoords(shadow))
		self._addNode(name, region, "shadow")
		for fovNode in self.fovNodes:
			fovComponent = self.nodes[fovNode]["region"]
			if Geometry.haveOverlappingEdge(fovComponent.polygon, region.polygon):
				self._addEdges(fovNode, name)
		return

	def _mergeShadow(self, existingShadow: ShadowRegion, shadow: Polygon):
		p = Geometry.union([existingShadow.polygon, shadow])
		region = ShadowRegion(existingShadow.name, Geometry.getPolygonCoords(p))
		self.nodes[existingShadow.name]["region"] = region
		self.nodes[existingShadow.name]["centroid"] = region.polygon.centroid
		return

	def _updateShadows(self, newShadow: Polygon):
		if len(self.shadowNodes) == 0:
			self._createNewShadow(newShadow)
		else:
			beforeUpdate = self.shadowNodes.copy()
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

	def _addSymbols(self, fovUnion: Union[Polygon, MultiPolygon]):
		for validatorName in self.validators:
			validator = self.validators[validatorName]
			if not validator.isRegion: continue
			validatorRegion: PolygonalRegion = validator.value
			insidePolys = Geometry.intersect(validatorRegion.polygon, fovUnion)
			insidePolys = [p for p in insidePolys if p.length > 0]
			insidePolys = list(filter(lambda p: not isinstance(p, LineString), insidePolys))
			for i in range(len(insidePolys)):
				poly = insidePolys[i]
				name = "%s-%d" % (validatorName, i)
				region = ValidatorRegion(name, Geometry.getPolygonCoords(poly), inFov=True)
				self._addNode(name, region, "symbol")
				broken = False
				for fovNode in self.fovNodes:
					fovRegion: SensingRegion = self.nodes[fovNode]["region"]
					if fovRegion.polygon.intersects(poly):
						self._addEdges(fovNode, name)
						broken = True
						break
				if broken: continue
			shadows = Geometry.shadows(validatorRegion.polygon, fovUnion)
			shadows = [p for p in shadows if p.length > 0]
			shadows = list(filter(lambda p: not isinstance(p, LineString), shadows))
			for i in range(len(shadows)):
				poly = shadows[i]
				name = "%s-%d" % (validatorName, (i + len(insidePolys)))
				region = ValidatorRegion(name, Geometry.getPolygonCoords(poly), inFov=False)
				self._addNode(name, region, "symbol")
				broken = False
				for shadowNode in self.shadowNodes:
					shadowRegion: SensingRegion = self.nodes[shadowNode]["region"]
					if shadowRegion.polygon.intersects(poly):
						self._addEdges(shadowNode, name)
						break
				if broken: continue
		return

	def _build(self, fovUnion: Union[Polygon, MultiPolygon]):
		self._addSensorRegionConnectedComponents(fovUnion)
		self._constructShadows(fovUnion)
		self._addSymbols(fovUnion)
		return

	def displayGraph(self, displayGeomGraph, displaySpringGraph):
		if displayGeomGraph:
			self._fig = GraphAlgorithms.displayGeometricGraph(self, self.fovNodes, self.shadowNodes)
		else:
			self._fig = GraphAlgorithms.displaySpringGraph(self, self.fovNodes, self.shadowNodes, self.symbolNodes)
		return

	def killDisplayedGraph(self):
		if self._fig:
			GraphAlgorithms.killDisplayedGraph(self._fig)
			self._fig = None
