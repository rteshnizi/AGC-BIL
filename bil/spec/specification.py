from bil.spec.symbol import SpecAction
from bil.spec.nfa import NFA

class Specification(object):
	def __init__(self, name, specificationDict):
		self.name = name
		self._specificationDict = specificationDict
		self._shapeIds = None
		self._regionSymbols = []
		self._symbolMap = self._buildSymbolMap()
		self.nfa = NFA(self.states, self.transitions, self._symbolMap)
		self.nfa.displayGraph()

	@property
	def states(self):
		return self._specificationDict["states"]

	@property
	def transitions(self):
		return self._specificationDict["transitions"]

	@property
	def _actionsDefs(self):
		return self._specificationDict["actions"]

	def __repr__(self):
		return "NFA-%s" % self.name

	def _buildSymbolMap(self):
		symbols = {}
		for symName in self._actionsDefs:
			symbols[symName] = SpecAction(symName, self._actionsDefs[symName])
			if symbols[symName].isRegion: self._regionSymbols.append(symName)
		return symbols

	def render(self, canvas):
		for symName in self._regionSymbols:
			region = self._symbolMap[symName].value
			region.render(canvas)
