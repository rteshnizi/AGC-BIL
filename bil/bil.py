import os
from bil.parser import EnvironmentParser, ObservationParser, SpecParser
from bil.gui.drawing import Drawing
from bil.gui.app import App
from bil.spec.lambdas import Prototypes

class Bil(object):
	def __init__(self, loadFromFile=False):
		Prototypes.initialize()
		self.loadFromFile = loadFromFile
		self.dataPrototype = "Prototype-2"
		# self.dataPrototype = "Prototype-1"
		# self.dataPrototype = "MovingSensor-0"
		Drawing.init(self.dataPrototype)
		self.mockDataDir = os.path.abspath(os.path.join("data", "Mock", self.dataPrototype))
		self.envParser = EnvironmentParser(self.mockDataDir)
		self.featureMap = None
		self.map = None
		(self.featureMap, self.map) = self.envParser.parse()
		if loadFromFile:
			self.obsParser = ObservationParser(self.mockDataDir)
			self.observations = self.obsParser.parseNew()
			# (self.scenario.observationOlds, self.scenario.agents) = self.obsParser.parse(self.map, self.scenario.fov)
		self.specParser = SpecParser(self.mockDataDir)
		self.specs = self.specParser.parse()
		self.app = App(self, self.update)
		self.previousObservation = None

	def run(self):
		self.app.mainloop()

	def update(self, observation):
		print("Updating specs with %s" % repr(observation))
		# for spec in self.specs:
		spec = self.specs[0]
		# spec.nfa.killDisplayedGraph()
		print("Validating specification %s" % repr(spec.name))
		spec.nfa.read(self.map, observation, self.previousObservation)
		self.previousObservation = observation
		spec.nfa.displayGraph()
		return
