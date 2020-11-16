import os
from bil.parser import EnvironmentParser, ObservationParser, SpecParser
from bil.model.scenario import Scenario
from bil.gui.app import App

class Bil(object):
	def __init__(self, loadFromFile=False):
		self.loadFromFile = loadFromFile
		self.mockDataDir = os.path.abspath(os.path.join("data", "Mock", "MovingSensor-0"))
		self.envParser = EnvironmentParser(self.mockDataDir)
		self.featureMap = None
		self.map = None
		(self.featureMap, self.map) = self.envParser.parse()
		self.scenario = Scenario(self.featureMap, self.map)
		if loadFromFile:
			self.obsParser = ObservationParser(self.mockDataDir)
			(self.scenario.observations, self.scenario.agents) = self.obsParser.parse(self.map, self.scenario.fov)
		self.specParser = SpecParser(self.mockDataDir)
		self.specs = self.specParser.parse()
		# self.spec = ["r38", "r72"]
		self.app = App(self.scenario, self.emulateUpdates, self.specs[0])

	def run(self):
		self.app.mainloop()

	def emulateUpdates(self):
		for observation in self.scenario.observations.values():
			isValid = observation.trajectory.validate(self.map)
			if not isValid:
				print("Trajectory of agent %s is invalid" % observation.agentName)
				return
			for spec in self.specs:
				print("validating specification %s against agent %s" % (repr(spec), observation.agentName))
				# isValid = observation.validate(self.map, self.fov, verbose=False)
				isValid = observation.validateWithSpecification(self.map, self.scenario.fov, spec)
				print("The specification %s is %s" % (repr(spec), "valid" if isValid else "invalid"))

	def update(self, data):
		print("BIL says: %s" % repr(data))
