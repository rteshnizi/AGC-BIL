import networkx as nx
import matplotlib.pyplot as plt
import re as RegEx
import json
from typing import Set

from bil.model.connectivityGraph import ConnectivityGraph
from bil.model.shadowTree import ShadowTree
from bil.observation.observations import Observation
from bil.spec.spaceTime import SpaceTimeSet
from bil.utils.graph import GraphAlgorithms
from bil.utils.geometry import Geometry

class Transition:
	def __init__(self, specifier, validators):
		self.specifier = specifier
		matches = RegEx.search(r"\((.*),\s+(.*)\)", self.specifier)
		self.name = matches.group(1)
		self.consuming = json.loads(matches.group(2).lower())
		self.validator = validators[self.name]

	def __repr__(self):
		return repr(self.specifier)

	def execute(self, pose):
		return self.validator.execute(pose)

class Penny:
	"""
	This is the weirdest name I could use.
	This represents a penny in what Dr. Shell referred to as a stack of pennies that will track a possible pose and its state
	"""
	def __init__(self, state, pose):
		self.state = state
		self.pose = pose

	def __repr__(self):
		return "(%s, %s)" % (self.state, self.pose)

	def __hash__(self):
		return hash(repr(self))

	def getShapelyPolygon(self, cGraph: ConnectivityGraph):
		cluster = cGraph.nodeToClusterMap[self.pose]
		return cGraph.nodeClusters[cluster].polygon

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
		self.activeStates: Set[Penny] = set()
		self._buildGraph()
		self._previousCGraph: ConnectivityGraph = None

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

	def read(self, envMap, observation: Observation, prevObservation):
		if len(observation.fov.sensors) == 0:
			print("Don't know how to handle no FOV yet")
			return
		cGraph = ConnectivityGraph(envMap, observation.fov, self.validators)
		condensedGraph = cGraph.condense()
		if prevObservation is None:
			if len(observation.tracks) == 0:
				for n in condensedGraph.nodes:
					if GraphAlgorithms.isShadowRegion(condensedGraph, n):
						self.activeStates.add(Penny(self.START_SYMBOL, n))
			elif len(observation.tracks) == 1:
				foundPolygon = False
				for n in condensedGraph.nodes:
					if GraphAlgorithms.isBeamNode(n): continue
					track = observation.tracks[next(iter(observation.tracks))]
					if Geometry.isXyInsidePolygon(track.pose.x, track.pose.y, cGraph.nodeClusters[n].polygon):
						self.activeStates.add(Penny(self.START_SYMBOL, n))
						foundPolygon = True
						break
				if not foundPolygon: raise RuntimeError("Track Pose %s is not inside any polygon." % repr(track.pose))
			else: raise RuntimeError("We only work with a single target for now.")
		else:
			# FIXME: Start here, and remove shadows that cannot be connected
			if len(observation.tracks) == 0:
				# Check connectivity with previous Graph (timed graph) for each penny -> Shadows that are connected will carry forward
				timedGraph = ShadowTree([self._previousCGraph, cGraph])
				self.propagatePennies(timedGraph)
			elif len(observation.tracks) == 1:
				# Check connectivity with previous Graph (timed graph) for each penny -> Shadows that are connected will to the observation carry forward and turn into the guy
				pass
			else: raise RuntimeError("We only work with a single target for now.")
		activeStatesCopy: Set[Penny] = self.activeStates.copy()
		while len(activeStatesCopy) > 0:
			penny = activeStatesCopy.pop()
			spaceSet = penny.getShapelyPolygon(cGraph)
			spaceTimeSet = SpaceTimeSet(spaceSet, observation.time)
			for outEdge in self.out_edges(penny.state):
				currentState = outEdge[0]
				nextState = outEdge[1]
				transition = self.get_edge_data(currentState, nextState)["transition"]
				passed = transition.execute(spaceTimeSet)
				if passed:
					if penny in self.activeStates: self.activeStates.remove(penny) # Non-consuming pennies are not in this set
					# While there are non-consuming transitions, we should keep doing traversing them.
					if not transition.consuming:
						activeStatesCopy.add(Penny(nextState, penny.pose))
					else:
						self.activeStates.add(Penny(nextState, penny.pose))
				else:
					self.activeStates.add(penny)
		self._previousCGraph = cGraph

	def propagatePennies(self, timedGraph: ShadowTree):
		for penny in self.activeStates:
			neighbors = timedGraph.getTemporalNeighbors(penny.pose)
			print(neighbors)

	def displayGraph(self):
		fig = plt.figure(len(GraphAlgorithms._allFigs))
		GraphAlgorithms._allFigs.add(fig)
		pos = nx.circular_layout(self)
		activeStates = { penny.state for penny in self.activeStates }
		inactiveStates = set(self.nodes) - activeStates
		nx.draw_networkx_nodes(self, pos, nodelist=activeStates, node_color="palegreen")
		nx.draw_networkx_nodes(self, pos, nodelist=inactiveStates - {"END"}, node_color="lightgrey")
		# END STATE
		nx.draw_networkx_nodes(self, pos, nodelist={"END"}, node_color="tomato" if "END" in inactiveStates else "palegreen")
		nx.draw_networkx_edges(self, pos)
		nx.draw_networkx_labels(self, pos, font_family="DejaVu Sans", font_size=10)
		edgeLabel = nx.get_edge_attributes(self, "transition")
		nx.draw_networkx_edge_labels(self, pos, edge_labels=edgeLabel)
		pennyText = [repr(penny) for penny in self.activeStates]
		plt.annotate("\n".join(pennyText), xy=(0.01, 0), xycoords="axes fraction")
		plt.axis("off")
		fig.show()
		self._fig = fig

	def killDisplayedGraph(self):
		if self._fig is not None:
			GraphAlgorithms.killDisplayedGraph(self._fig)
			self._fig = None
