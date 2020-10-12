import networkx as nx
import matplotlib.pyplot as plt
import time
from mpl_toolkits.mplot3d import Axes3D
from math import nan, isnan
from queue import Queue
from typing import List, Tuple

class GraphAlgorithms:
	_allFigs = set()

	@staticmethod
	def isConnectedBfs(graph: nx.DiGraph, start, end, premittedBeams = []):
		q = Queue()
		visited = set()
		q.put([start])
		while not q.empty():
			path = q.get()
			n = path[-1]
			visited.add(n)
			if n == end: return path
			if (n != start) and (GraphAlgorithms.isBeamNode(n)) and (n not in premittedBeams): continue
			for child in graph.adj[n]:
				if child in visited: continue
				newPath = list(path)
				newPath.append(child)
				q.put(newPath)
		return None

	@staticmethod
	def chainGraphs(graphs: List[nx.DiGraph], startInd = 0, endInd = nan) -> nx.DiGraph:
		if isnan(endInd): endInd = len(graphs) - 1
		if startInd == endInd: return nx.DiGraph(graphs[startInd])
		if not startInd < endInd: raise IndexError("startIndex must be less than endIndex")
		chained = graphs[startInd]
		for i in range(startInd + 1, endInd + 1):
			chained = nx.compose(chained, graphs[i])
		return chained

	@staticmethod
	def getBeamLeaves(graph: nx.DiGraph):
		leaves = []
		for n in graph.nodes():
			if GraphAlgorithms.isBeamNode(n) and graph.out_degree(n) == 0:
				leaves.append(n)
		return leaves

	@staticmethod
	def getBeamName(passingLane, notPassingLane):
		return "%s¦|%s" % (passingLane, notPassingLane)

	@staticmethod
	def isBeamNode(n: str) -> bool:
		return "¦|" in n

	@staticmethod
	def getBeamEnds(n: str) -> Tuple[str, str]:
		parts = n.split("¦|")
		return (parts[0], parts[1])

	@staticmethod
	def cloneNodeProps(frm: dict, to: dict) -> bool:
		for key in frm: to[key] = frm[key]

	@staticmethod
	def displayGraphAuto(g: nx.DiGraph, displayGeomGraph, displaySpringGraph):
		"""
		Detects beam and non-beam nodes for the correct coloring
		"""
		bNodes = []
		rNodes = []
		for n in g.nodes:
			if GraphAlgorithms.isBeamNode(n):
				rNodes.append(n)
			else:
				bNodes.append(n)
		if displayGeomGraph:
			GraphAlgorithms.displayGeometricGraph(g, bNodes, rNodes)
		if displaySpringGraph:
			GraphAlgorithms.displaySpringGraph(g, bNodes, rNodes)

	@staticmethod
	def displaySpringGraph(g: nx.DiGraph, blueNodes, redNodes):
		fig = plt.figure(len(GraphAlgorithms._allFigs))
		GraphAlgorithms._allFigs.add(fig)
		pos = nx.spring_layout(g)
		nx.draw_networkx_nodes(g, pos, nodelist=blueNodes, node_color='palegreen')
		nx.draw_networkx_nodes(g, pos, nodelist=redNodes, node_color='tomato')
		nx.draw_networkx_edges(g, pos)
		nx.draw_networkx_labels(g, pos, font_family="DejaVu Sans", font_size=10)
		plt.axis("off")
		fig.show()
		return fig

	@staticmethod
	def getNodeCoordinates(g, nodeName) -> tuple:
		return (g.nodes[nodeName]["centroid"].x, g.nodes[nodeName]["centroid"].y, g.nodes[nodeName]["timestamp"])

	@staticmethod
	def displayGeometricGraph(g: nx.DiGraph, blueNodes, redNodes):
		fig = plt.figure(len(GraphAlgorithms._allFigs))
		GraphAlgorithms._allFigs.add(fig)
		ax = fig.gca(projection='3d')
		xMap = []
		yMap = []
		zMap = []
		xShadow = []
		yShadow = []
		zShadow = []
		xOther = []
		yOther = []
		zOther = []
		labelsMap = []
		labelsShadow = []
		labelsOther = []
		edgesToAdd = set()
		for node in g.nodes:
			if GraphAlgorithms.isBeamNode(node): continue
			if "type" not in g.nodes[node]:
				xOther.append(g.nodes[node]["centroid"].x)
				labelsOther.append(node)
				yOther.append(g.nodes[node]["centroid"].y)
				zOther.append(g.nodes[node]["timestamp"])
			elif g.nodes[node]["type"] == "s":
				xShadow.append(g.nodes[node]["centroid"].x)
				labelsShadow.append(node)
				yShadow.append(g.nodes[node]["centroid"].y)
				zShadow.append(g.nodes[node]["timestamp"])
			else:
				xMap.append(g.nodes[node]["centroid"].x)
				labelsMap.append(node)
				yMap.append(g.nodes[node]["centroid"].y)
				zMap.append(g.nodes[node]["timestamp"])
			for neighbor in g.adj[node]:
				if GraphAlgorithms.isBeamNode(neighbor): continue
				if g.nodes[node]["timestamp"] == g.nodes[neighbor]["timestamp"]: continue
				higher = node if g.nodes[node]["timestamp"] > g.nodes[neighbor]["timestamp"] else neighbor
				lower = node if higher != node else neighbor
				edgesToAdd.add((GraphAlgorithms.getNodeCoordinates(g, lower), GraphAlgorithms.getNodeCoordinates(g, higher)))
		ax.plot(xMap, yMap, zMap, "ro")
		ax.plot(xShadow, yShadow, zShadow, "bo")
		ax.plot(xOther, yOther, zOther, "go")
		for e in edgesToAdd:
			xs = [e[0][0], e[1][0]]
			ys = [e[0][1], e[1][1]]
			zs = [e[0][2], e[1][2]]
			ax.plot(xs, ys, zs, "g-")
		for i in range(len(labelsMap)):
			ax.text(xMap[i], yMap[i], zMap[i], labelsMap[i], color="darkred")
		for i in range(len(labelsShadow)):
			ax.text(xShadow[i], yShadow[i], zShadow[i], labelsShadow[i], color="darkblue")
		for i in range(len(labelsOther)):
			ax.text(xOther[i], yOther[i], zOther[i], labelsOther[i], color="darkgreen")
		ax.autoscale()
		fig.show()
		return fig

	@staticmethod
	def _getTimedNodeName(nodeName, timestamp):
		return "%s-%.1f" % (nodeName, timestamp)

	@staticmethod
	def chainGraphsThroughTime(graphs) -> nx.DiGraph:
		chained = nx.DiGraph()
		for g in graphs:
			print ("Chaining for %.1f" % g.timestamp)
			for n in g.nodes:
				newNode = GraphAlgorithms._getTimedNodeName(n, g.timestamp)
				chained.add_node(newNode)
				GraphAlgorithms.cloneNodeProps(g.nodes[n], chained.nodes[newNode])
				chained.nodes[newNode]["polygonName"] = n
			for e in g.edges:
				chained.add_edge(GraphAlgorithms._getTimedNodeName(e[0], g.timestamp), GraphAlgorithms._getTimedNodeName(e[1], g.timestamp))

		print("n = %d" % len(chained.nodes))
		print("O(%d)" % (len(chained.nodes) * len(chained.nodes)))
		for n1 in chained.nodes:
			for n2 in chained.nodes:
				if chained.nodes[n1]["polygonName"] != chained.nodes[n2]["polygonName"]: continue
				higher = n1 if chained.nodes[n1]["timestamp"] > chained.nodes[n2]["timestamp"] else n2
				lower = n1 if higher != n1 else n2
				chained.add_edge(lower, higher)
		return chained

	@staticmethod
	def chainCondensedGraphsThroughTime(graphs: List["ConnectivityGraph"], startInd = 0, endInd = None) -> nx.DiGraph:
		if endInd is None: endInd = len(graphs)
		if (startInd - endInd) >= 0:
			startInd = 0
			endInd = len(graphs)
		chained = nx.DiGraph()
		nodeLayers = []
		for i in range(startInd, endInd):
			g = graphs[i]
			layerIndex = len(nodeLayers)
			nodeLayers.append([])
			print ("Chaining for %.1f" % g.timestamp)
			for n in g.condensed.nodes:
				newNode = GraphAlgorithms._getTimedNodeName(n, g.timestamp)
				chained.add_node(newNode)
				n = n if n in g.nodes else g.condensed.nodes[n]["mappedName"]
				GraphAlgorithms.cloneNodeProps(g.nodes[n], chained.nodes[newNode])
				chained.nodes[newNode]["polygonName"] = n # This is lazy way to obtain the name of the polygon without doing string split etc.
				chained.nodes[newNode]["cluster"] = g.nodeClusters[g.nodeToClusterMap[n]]
				nodeLayers[layerIndex].append(newNode)
			for e in g.condensed.edges:
				chained.add_edge(GraphAlgorithms._getTimedNodeName(e[0], g.timestamp), GraphAlgorithms._getTimedNodeName(e[1], g.timestamp))

		print("n = %d" % len(chained.nodes))
		print("O(%d)" % (len(chained.nodes) * len(chained.nodes)))
		startTime = time.time()
		for layerIndex in range(len(nodeLayers) - 1):
			lowerLayer = nodeLayers[layerIndex]
			upperLayer = nodeLayers[layerIndex + 1]
			for n1 in lowerLayer:
				for n2 in upperLayer:
					shouldConnect = False
					if chained.nodes[n1]["polygonName"] == chained.nodes[n2]["polygonName"]: shouldConnect = True
					if not shouldConnect and chained.nodes[n1]["polygonName"] in chained.nodes[n1]["cluster"]: shouldConnect = True
					if not shouldConnect: continue
					higher = n1 if chained.nodes[n1]["timestamp"] > chained.nodes[n2]["timestamp"] else n2
					lower = n1 if higher != n1 else n2
					chained.add_edge(lower, higher)

		# All Layers
		# for n1 in chained.nodes:
		# 	for n2 in chained.nodes:
		# 		if chained.nodes[n1]["polygonName"] != chained.nodes[n2]["polygonName"]: continue
		# 		higher = n1 if chained.nodes[n1]["timestamp"] > chained.nodes[n2]["timestamp"] else n2
		# 		lower = n1 if higher != n1 else n2
		# 		chained.add_edge(lower, higher)
		endTime = time.time()
		print("%.3f s" % (endTime - startTime))
		return chained

	@staticmethod
	def killDisplayedGraph(fig):
		plt.close(fig)
		GraphAlgorithms._allFigs.remove(fig)

	@staticmethod
	def killAllDisplayedGraph():
		for fig in GraphAlgorithms._allFigs:
			plt.close(fig)
		GraphAlgorithms._allFigs.clear()
