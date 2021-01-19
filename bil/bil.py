import os
from bil.parser import EnvironmentParser, ObservationParser, SpecParser
from bil.model.scenario import Scenario
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
		self.app = App(self, self.emulateUpdates)

	def run(self):
		self.app.mainloop()

	def emulateUpdates(self):
		previousObservation = None
		for t in sorted(self.observations):
			observation = self.observations[t]
			# for spec in self.specs:
			spec = self.specs[0]
			print("Validating specification %s" % repr(spec.name))
			# isValid = observation.validate(self.map, self.fov, verbose=False)
			spec.nfa.read(self.map, observation, previousObservation)
			previousObservation = observation
			return
			# isValid = observation.validateWithSpecification(self.map, self.scenario.fov, spec)
			# print("The specification %s is %s" % (repr(spec), "valid" if isValid else "invalid"))

	def update(self, data):
		print("BIL says: %s" % repr(data))
