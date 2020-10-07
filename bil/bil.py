import os
from bil.parser import Parser
from bil.model.scenario import Scenario
from bil.gui.app import App

class Bil(object):
	def __init__(self, loadFromFile=False):
		self.loadFromFile = loadFromFile
		self.mockDataDir = os.path.abspath(os.path.join("data", "Mock", "MovingSensor-0"))
		self._observationsJsonPath = os.path.abspath(os.path.join(self.mockDataDir, "obs.json"))
		self.parser = Parser(self.mockDataDir)
		self.featureMap = None
		self.map = None
		(self.featureMap, self.map) = self.parser.parse()
		self.scenario = Scenario(self.featureMap, self.map, self._observationsJsonPath)
		self.app = App(self.scenario)

	def run(self):
		self.app.mainloop()

	def update(self, data):
		print("BIL says: %s" % repr(data))
