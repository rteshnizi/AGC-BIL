import networkx as nx
import matplotlib.pyplot as plt
from bil.utils.graph import GraphAlgorithms

class NFA(nx.DiGraph):
	def __init__(self, nfaSpecifier):
		super().__init__()
		self._nfaSpecifier = nfaSpecifier
		self.START_SYMBOL = "START"
		self.TERMINAL_SYMBOL = "END"
		self.EDGE_FUNCTION = "FUNC"
		self._fig = None
		self._buildGraph()

	def _buildGraph(self):
		self.add_node(self.START_SYMBOL)
		previousState = self.START_SYMBOL
		stateCounter = 0
		for i in range(len(self._nfaSpecifier)):
			symbol = self._nfaSpecifier[i]
			nextState = "S-%d" % stateCounter if i < len(self._nfaSpecifier) - 1 else self.TERMINAL_SYMBOL
			self.add_edge(previousState, nextState, function=symbol)
			previousState = nextState
			stateCounter += 1

	def read(stringToRead):
		pass

	def displayGraph(self):
		fig = plt.figure(len(GraphAlgorithms._allFigs))
		GraphAlgorithms._allFigs.add(fig)
		pos = nx.spring_layout(self)
		nx.draw_networkx_nodes(self, pos, node_color='palegreen')
		nx.draw_networkx_edges(self, pos)
		nx.draw_networkx_labels(self, pos, font_family="DejaVu Sans", font_size=10)
		edgeLabel = nx.get_edge_attributes(self, self.EDGE_FUNCTION)
		nx.draw_networkx_edge_labels(self, pos, labels = edgeLabel)
		plt.axis("off")
		fig.show()
		self._fig = fig

	def killDisplayedGraph(self):
		if self._fig is not None:
			GraphAlgorithms.killDisplayedGraph(self._fig)
			self._fig = None

	# def lambdaC(self, p):
	# 	return (Lambda(self, p), True)

	# def lambdaA(self, p):
	# 	return (Lambda(self, p), False)

	# "specifier":

			# "C": "lambdaC(p) : Geometry.isInside(p, [[-10, 0], [-10, -50], [-60, 0]])",
			# "T1": "lambdaA(p) : True",
			# "T2": "lambdaA(p) : timeQuery(p, 2.5, True, False)"
