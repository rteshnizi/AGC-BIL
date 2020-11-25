import networkx as nx
import matplotlib.pyplot as plt
import re as RegEx
from bil.utils.graph import GraphAlgorithms

class Transition:
	def __init__(self, specifier):
		self.specifier = specifier

	def __repr__(self):
		return self.specifier

class NFA(nx.DiGraph):
	def __init__(self, states, transitions, actions):
		super().__init__()
		self.states = states
		self.transitions = transitions
		self.actions = actions
		self.START_SYMBOL = "START"
		self.TERMINAL_SYMBOL = "END"
		self._fig = None
		self.activeStates = set()
		self._buildGraph()
		self.activeStates.add(self.START_SYMBOL)

	def _buildGraph(self):
		if self.START_SYMBOL not in self.states: raise RuntimeError("%s is not among the states." % self.START_SYMBOL)
		if self.TERMINAL_SYMBOL not in self.states: raise RuntimeError("%s is not among the states." % self.TERMINAL_SYMBOL)
		self.add_nodes_from(self.states)
		for fromState in self.transitions:
			for i in range(len(self.transitions[fromState])):
				matches = RegEx.search(r"\((\(.*\)),\s+(.*)\)", self.transitions[fromState][i])
				transition = Transition(matches.group(1))
				toState = matches.group(2)
				self.add_edge(fromState, toState, transition=transition)

	def read(self, envMap, observation):
		if len(observation.tracks) == 0: return


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
