import os
from bil.parser import EnvironmentParser, ObservationParser, SpecParser
from bil.model.scenario import Scenario
from bil.model.fieldOfView import FieldOfView
from bil.gui.app import App

class Bil(object):
	def __init__(self, loadFromFile=False):
		self.loadFromFile = loadFromFile
		self.mockDataDir = os.path.abspath(os.path.join("data", "Mock", "Prototype-1"))
		# self.mockDataDir = os.path.abspath(os.path.join("data", "Mock", "MovingSensor-0"))
		self.envParser = EnvironmentParser(self.mockDataDir)
		self.featureMap = None
		self.map = None
		(self.featureMap, self.map) = self.envParser.parse()
		if loadFromFile:
			self.obsParser = ObservationParser(self.mockDataDir)
			self.observations = self.obsParser.parseNew()
			self.fieldOfView = FieldOfView(self.map)
			self.addRegionsToFieldOfView()
			# (self.scenario.observationOlds, self.scenario.agents) = self.obsParser.parse(self.map, self.scenario.fov)
		self.specParser = SpecParser(self.mockDataDir)
		self.specs = self.specParser.parse()
		self.app = App(self, self.emulateUpdates, self.specs[0])

	def addRegionsToFieldOfView(self):
		for observationId in self.observations:
			observation = self.observations[observationId]
			# FIXME: For now there is only one sensor per vehicle, we might have to union them later
			for sensorId in observation.sensors:
				sensor = observation.sensors[sensorId]
				self.fieldOfView.append(sensor.fov.region, observationId, sensorId)

	def run(self):
		self.app.mainloop()

	def emulateUpdates(self):
		previousObservation = None
		for t in sorted(self.scenario.observations):
			observation = self.scenario.observations[t]
			for spec in self.specs:
				print("validating specification %s" % repr(spec.name))
				# isValid = observation.validate(self.map, self.fov, verbose=False)
				spec.nfa.read(self.map, observation, previousObservation)
				previousObservation = observation
				# isValid = observation.validateWithSpecification(self.map, self.scenario.fov, spec)
				# print("The specification %s is %s" % (repr(spec), "valid" if isValid else "invalid"))

	def update(self, data):
		print("BIL says: %s" % repr(data))
