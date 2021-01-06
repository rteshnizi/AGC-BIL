import networkx as nx
import matplotlib.pyplot as plt
import re as RegEx
import json
from bil.utils.graph import GraphAlgorithms
from bil.utils.geometry import Geometry
from bil.model.connectivityGraph import ConnectivityGraph

class Transition:
	def __init__(self, specifier, validators):
		self.specifier = specifier
		matches = RegEx.search(r"\((.*),\s+(.*)\)", self.specifier)
		self.name = matches.group(1)
		self.consuming = json.loads(matches.group(2).lower())
		self.validator = validators[self.name]

	def __repr__(self):
		return self.specifier

class NFA(nx.DiGraph):
	def __init__(self, specName, states, transitions, validators):
		super().__init__()
		self._specName = specName
		self.states = states
		self.transitions = transitions
		self.validators = validators
		self.START_SYMBOL = "START"
		self.TERMINAL_SYMBOL = "END"
		self._fig = None
		self.activeStates = set()
		self._buildGraph()
		self.activeStates.add(self.START_SYMBOL)
		self.visitedStates = set()
		self._possiblePositions = set()
		self._graphCounter = 0

	def __repr__(self):
		return "%s.NFA" % self._specName

	def _buildGraph(self):
		if self.START_SYMBOL not in self.states: raise RuntimeError("%s is not among the states." % self.START_SYMBOL)
		if self.TERMINAL_SYMBOL not in self.states: raise RuntimeError("%s is not among the states." % self.TERMINAL_SYMBOL)
		self.add_nodes_from(self.states)
		for fromState in self.transitions:
			for i in range(len(self.transitions[fromState])):
				matches = RegEx.search(r"\((\(.*\)),\s+(.*)\)", self.transitions[fromState][i])
				transition = Transition(matches.group(1), self.validators)
				toState = matches.group(2)
				self.add_edge(fromState, toState, transition=transition)

	def read(self, envMap, observation, prevObservation):
		if len(observation.fov.sensors) == 0:
			print("Don't know how to handle no FOV yet")
			return
		cGraph = ConnectivityGraph(envMap, observation.fov, self._graphCounter)
		self._graphCounter += 1
		condensedGraph = cGraph.condense(self)
		GraphAlgorithms.displayGraphAuto(condensedGraph, True, True)
		return
		if prevObservation is None:
			if len(observation.tracks) == 0:
				for n in condensedGraph.nodes:
					if GraphAlgorithms.isShadowRegion(condensedGraph, n): self._possiblePositions.add(n)
			if len(observation.tracks) > 1: raise RuntimeError("We only work with a single target for now.")
			self._possiblePositions.add(n)
		newActiveStates = self.activeStates.copy()
		for state in self.activeStates:
			for outboundEdge in self.edges(state):
				transition = self.get_edge_data(outboundEdge[0], outboundEdge[1])
				for p in self._possiblePositions:
					passed = transition.execute(p)
					nextState = outboundEdge[1] if passed else outboundEdge[0]
					newActiveStates.remove(state)
					newActiveStates.add(nextState)
		pass

	def displayGraph(self):
		fig = plt.figure(len(GraphAlgorithms._allFigs))
		GraphAlgorithms._allFigs.add(fig)
		pos = nx.spring_layout(self)
		nx.draw_networkx_nodes(self, pos, node_color='palegreen')
		nx.draw_networkx_edges(self, pos)
		nx.draw_networkx_labels(self, pos, font_family="DejaVu Sans", font_size=10)
		edgeLabel = nx.get_edge_attributes(self, "transition")
		nx.draw_networkx_edge_labels(self, pos, labels = edgeLabel)
		plt.axis("off")
		fig.show()
		self._fig = fig

	def killDisplayedGraph(self):
		if self._fig is not None:
			GraphAlgorithms.killDisplayedGraph(self._fig)
			self._fig = None
