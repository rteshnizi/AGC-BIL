import networkx as nx
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point, Polygon
from typing import List, Set, Dict

from bil.model.observationOld import ObservationOld
from bil.model.polygonalRegion import PolygonalRegion
from bil.model.shadowRegion import ShadowRegion
from bil.utils.geometry import Geometry
from bil.utils.graph import GraphAlgorithms

class ConnectivityGraph(nx.DiGraph):
	def __init__(self, map, fov, timestamp, graphIndex):
		super().__init__()
		self._regionNodes = []
		self._beamNodes = []
		self._beamSet = set()
		self.fov = fov
		self.map = map
		self.timestamp = timestamp
		self.graphIndex = graphIndex
		self._disjointPolys: List[PolygonalRegion] = []
		print("Building graph for %s" % self.timestamp)
		self._buildFromMap(fov)
		self._fig = None
		self._OBSOLETE_condensed = None
		self._OBSOLETE_nodeClusters: Dict[str, Set[str]] = {}
		self._OBSOLETE_nodeToClusterMap: Dict[str, str] = {}

	def __repr__(self):
		return "cGraph-%.2f" % self.timestamp

	def _addCluster(self, node):
		self.nodeClusters[node] = { node }
		self.nodeToClusterMap[node] = node

	def _condenseBeam(self, beam):
		(start, end) = GraphAlgorithms.getBeamEnds(beam)
		if start in self.nodeToClusterMap:
			start = self.nodeToClusterMap[start]
		if end in self.nodeToClusterMap:
			end = self.nodeToClusterMap[end]
		return GraphAlgorithms.getBeamName(start, end)

	def condense(self, nfa):
		print("Condensing %s" % self.timestamp)
		self.nodeClusters: Dict[str, Set[str]] = {}
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
						self.nodeClusters[self.nodeToClusterMap[n1]].add(n2)
						self.nodeToClusterMap[n2] = self.nodeToClusterMap[n1]
					elif n2 in self.nodeToClusterMap:
						self.nodeClusters[self.nodeToClusterMap[n2]].add(n1)
						self.nodeToClusterMap[n1] = self.nodeToClusterMap[n2]
					else:
						self._addCluster(n1)
						self.nodeClusters[self.nodeToClusterMap[n1]].add(n2)
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
		for validator in nfa.validators.values():
			if validator.isRegion:
				region1 = validator.value
				condensedGraph.add_node(region1.name)
				condensedGraph.nodes[region1.name]["timestamp"] = self.timestamp
				condensedGraph.nodes[region1.name]["graphIndex"] = self.graphIndex
				condensedGraph.nodes[region1.name]["region"] = region1
				condensedGraph.nodes[region1.name]["centroid"] = region1.polygon.centroid
				condensedGraph.nodes[region1.name]["mappedName"] = region1.name
				self._addCluster(region1.name)
				for n in self.nodeToClusterMap:
					if GraphAlgorithms.isBeamNode(n): continue
					region2 = self.nodes[n]["region"]
					if Geometry.polygonAndPolygonIntersect(region1.polygon, region2.polygon):
						condensedGraph.nodes[region1.name]["type"] = "s" if isinstance(region2, ShadowRegion) else "m"
						condensedGraph.add_edge(region1.name, self.nodeToClusterMap[n])
						break

		print(len(self.nodes))
		print(len(condensedGraph.nodes))
		return condensedGraph

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
		self.nodes[nodeName]["graphIndex"] = self.graphIndex

	def _addRegion(self, polygonalRegion):
		name = polygonalRegion.name
		self._addNode(name)
		self.nodes[name]["region"] = polygonalRegion
		self.nodes[name]["centroid"] = polygonalRegion.polygon.centroid
		self.nodes[name]["type"] = "s" if isinstance(polygonalRegion, ShadowRegion) else "m"
		self._regionNodes.append(name)

	def _addBeam(self, name):
		self._addNode(name)
		self._beamNodes.append(name)

	def _getMapRegion(self, name):
		return self.map.regions.get(name.split("-")[0], None)

	def _buildFromMap(self, fov):
		fovUnion = Geometry.union([r.polygon for r in fov])
		for fovRegion in fov: self._addRegion(fovRegion)
		for mapR in self.map.regions.values():
			if Geometry.polygonAndPolygonIntersect(mapR.polygon, fovUnion):
				insidePolys = Geometry.intersect(mapR.polygon, fovUnion)
				insidePolys = [p for p in insidePolys if p.length > 0]
				shadows = Geometry.nonOverlapping(mapR.polygon, fovUnion)
				shadows = [p for p in shadows if p.length > 0]
				self._disjointPolys += [PolygonalRegion("%s-%d" % (mapR.name, i), list(zip(*(insidePolys[i].exterior.coords.xy)))) for i in range(len(insidePolys))]
				self._disjointPolys += [ShadowRegion("%s-%d" % (mapR.name, i + len(insidePolys)), list(zip(*(shadows[i].exterior.coords.xy)))) for i in range(len(shadows))]
			else:
				self._disjointPolys.append(ShadowRegion("%s-0" % (mapR.name), list(zip(*(mapR.polygon.exterior.coords.xy)))))

		for p1 in self._disjointPolys:
			if not isinstance(p1, ShadowRegion): continue
			if self._getMapRegion(p1.name).isObstacle: continue
			self._addRegion(p1)
			for fovRegion in fov:
				e = Geometry.commonEdge(p1.polygon, fovRegion.polygon)
				if e: self._addBeamNodes(p1.name, fovRegion.name)
			for p2 in self._disjointPolys:
				if p1 == p2: continue
				if not isinstance(p2, ShadowRegion): continue
				if self._getMapRegion(p2.name).isObstacle: continue
				e = Geometry.commonEdge(p1.polygon, p2.polygon)
				if e: self.add_edge(p1.name, p2.name)

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
			self._fig = GraphAlgorithms.displayGeometricGraph(self, self._regionNodes, self._beamNodes)
		else:
			self._fig = GraphAlgorithms.displaySpringGraph(self, self._regionNodes, self._beamNodes)

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
