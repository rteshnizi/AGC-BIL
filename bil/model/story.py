import networkx as nx
from typing import List

from bil.model.map import Map
from bil.model.trajectory import Trajectory
from bil.utils.geometry import Geometry
from bil.utils.graph import GraphAlgorithms

class Story:
	def __init__(self, agent, trajectoryData, envMap: Map, valid: list):
		self.agent = agent
		self.trajectory = Trajectory("T%s" % self.agent.name, trajectoryData)
		self.sensed = self.trajectory.clip(valid)
		self.groundTruth: list = None

	def getSensorReadings(self, graph):
		sensorReadings = []
		for i in range(len(self.sensed.poses) - 1):
			readings = graph.getSensorReadings(self.sensed.poses[i], self.sensed.poses[i + 1])
			sensorReadings = sensorReadings + readings
		return sensorReadings

	def render(self, canvas):
		print("Traj %s" % self.agent.name)
		self.trajectory.render(canvas)

	def validate(self, envMap: Map, fov: "FieldOfView", verbose=False) -> bool:
		if not self.trajectory.validate(envMap):
			print("invalid trajectory %s" % self.trajectory.name)
			return False

		# FIXME: For now the assumption is that fov is static, so we use fov.cGraphs[0]
		graph = fov.cGraphs[0]
		self.groundTruth = self.trajectory.buildString(graph)

		totalIsValid = False
		longestLength = 0
		longest = None
		for i in range(len(self.groundTruth)):
			for j in range(i + 1, len(self.groundTruth)):
				isValid = self.validateWithSpecification(envMap, fov, graph, self.groundTruth[i:j], sensorReadings, verbose)
				if isValid:
					totalIsValid = True
					print("validated %s" % repr(self.groundTruth[i:j]))
					if j - i > longestLength:
						longest = self.groundTruth[i:j]
						longestLength = j - i
		# isValid = self.validateWithSpecification(envMap, fov, graph, self.groundTruth[3:6], sensorReadings, verbose)
		# if isValid: return True
		return totalIsValid

	def validateWithSpecification(self, map: Map, fov: "FieldOfView", graph, specification, sensorReadings, verbose=False) -> bool:
		if len(specification) == 0: return False

		subGraphs = []
		numSensorReadings = len(sensorReadings)
		for i in range(numSensorReadings + 1):
			start = specification[0] if i == 0 else sensorReadings[i - 1]
			end = None if i == numSensorReadings else sensorReadings[i]
			subGraphs.append(self.getSubGraph(graph, specification, start, end))
		if not self.canChain(subGraphs): return False

		isValid = self.validateChainGraph(specification, sensorReadings, subGraphs)
		return isValid

	def getSubGraph(self, graph: nx.DiGraph, groundTruth, start, end):
		subGraph = nx.DiGraph()
		searchNodes = set(groundTruth)
		searchNodes.add(start)
		for node1 in searchNodes:
			subGraph.add_node(node1)
			for node2 in searchNodes:
				if node1 == node2: continue
				path = GraphAlgorithms.isConnectedBfs(graph, node1, node2)
				if path:
					subGraph.add_node(node2)
					subGraph.add_edge(node1, node2)

		subGraph = nx.algorithms.bfs_tree(subGraph, start)

		if not end: return subGraph

		toAdd = set()
		for node in subGraph.nodes:
			path = GraphAlgorithms.isConnectedBfs(graph, node, end)
			if path:
				toAdd.add(node)
		if len(toAdd) == 0: return subGraph

		subGraph.add_node(end)
		for node in toAdd:
			subGraph.add_edge(node, end)
		return subGraph

	def canChain(self, subGraphs) -> bool:
		leavesByGraph = []
		for g in subGraphs:
			leavesByGraph.append(GraphAlgorithms.getBeamLeaves(g))

		for i in range(len(subGraphs) - 1):
			if len(leavesByGraph[i]) == 0: return False
			for n in leavesByGraph[i]:
				if not subGraphs[i + 1].has_node(n):
					return False
		return True

	def validateChainGraph(self, groundTruth, sensorReadings, subGraphs: List[nx.DiGraph]) -> bool:
		# FIXME: for Dynamic Programming we can store the chained graphs as needed
		for i in range(1, len(groundTruth)):
			# Each subgraph represents a sensor reading
			for j in range(len(subGraphs)):
				for k in range(j + 1):
					p_i_1 = groundTruth[i - 1]
					p_i = groundTruth[i]
					chained = GraphAlgorithms.chainGraphs(subGraphs, k, j)
					if p_i_1 not in chained.nodes: continue
					if p_i not in chained.nodes: continue
					# FIXME: Once we have dynamic programming we wouldn't have to actually search for paths
					permittedBeams = sensorReadings[k:j]
					if not GraphAlgorithms.isConnectedBfs(chained, p_i_1, p_i, permittedBeams): continue

					if i == len(groundTruth) - 1 and j == len(sensorReadings):
						GraphAlgorithms.displayGraphAuto(chained)
						return True
		return False
