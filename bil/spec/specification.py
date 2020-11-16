from bil.spec.symbol import SpecSymbol
from bil.spec.nfa import NFA

class Specification(object):
	def __init__(self, specificationDict):
		self._specificationDict = specificationDict
		self._shapeIds = None
		self._regionSymbols = []
		self._symbolMap = self._buildSymbolMap()
		self.nfa = NFA(self.specifier)
		self.nfa.displayGraph()

	@property
	def specifier(self):
		return self._specificationDict["specifier"]

	@property
	def _symbolDefs(self):
		return self._specificationDict["symbols"]

	def __repr__(self):
		return repr(self.specifier)

	def _buildSymbolMap(self):
		symbols = {}
		for symName in self._symbolDefs:
			symbols[symName] = SpecSymbol(symName, self._symbolDefs[symName])
			if symbols[symName].isRegion: self._regionSymbols.append(symName)
		return symbols

	def render(self, canvas):
		for symName in self._regionSymbols:
			region = self._symbolMap[symName].value
			region.render(canvas)
