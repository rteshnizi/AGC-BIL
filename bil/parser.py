from abc import ABC
from typing import Tuple
import json
import os

from bil.model.featureMap import FeatureMap
from bil.model.map import Map
from bil.model.observation import Observation
from bil.model.agent import Agent
from bil.model.teamMember import TeamMember
from bil.model.observation import Observation

class Parser(ABC):
	def __init__(self, dirAbsPath):
		self._dirAbsPath = dirAbsPath

class EnvironmentParser(Parser):
	def __init__(self, dirAbsPath):
		super().__init__(dirAbsPath)
		self._fmPath = os.path.abspath(os.path.join(self._dirAbsPath, "fm.json"))
		self._saMapPath = os.path.abspath(os.path.join(self._dirAbsPath, "sam.json"))
		self._ptListFile = os.path.abspath(os.path.join(self._dirAbsPath, "dt.json"))

	def parse(self) -> Tuple[object, object]:
		"""
		Returns
		===
		A tuple `(featureMap, envMap)`
		"""
		featureMap = self._buildFeatureMap()
		envMap = self._buildMap(featureMap)
		return (featureMap, envMap)

	def _buildFeatureMap(self):
		if not os.path.isfile(self._fmPath):
			raise FileNotFoundError("%s does not exist" % self._fmPath)
		with open(self._fmPath, 'r') as jsonFile:
			self._parsedFm = json.load(jsonFile)
		return FeatureMap(self._parsedFm["Nodes"])

	def _buildMap(self, featureMap) -> Map:
		if not os.path.isfile(self._saMapPath):
			raise FileNotFoundError("%s does not exist" % self._saMapPath)
		if not os.path.isfile(self._ptListFile):
			raise FileNotFoundError("%s does not exist" % self._ptListFile)
		with open(self._saMapPath, 'r') as jsonFile:
			self._parsedSaMap = json.load(jsonFile)
		with open(self._ptListFile, 'r') as jsonFile:
			self._parsedPtList = json.load(jsonFile)
		return Map(self._parsedPtList["Points"], [n["vertexIDs"] for n in self._parsedSaMap["Nodes"]], [n["feature"] for n in self._parsedSaMap["Nodes"]], featureMap)

class ObservationParser(Parser):
	def __init__(self, dirAbsPath):
		super().__init__(dirAbsPath)
		self._observationsJsonPath = os.path.abspath(os.path.join(self._dirAbsPath, "obs.json"))

	def parse(self, envMap, fov):
		"""
		Returns
		===
		A tuple `(observations, sens)`
		Stories are sensor readings, Agents are sensor locations
		"""
		with open(self._observationsJsonPath, 'r') as jsonFile:
			parsedObservation = json.load(jsonFile)
		idNum = 0
		observations = {}
		agents = {}
		for entity in parsedObservation:
			idNum += 1
			# Not a team member
			if "valid" in entity:
				trajectoryData = entity["Datap"]
				valid = entity["valid"]
				s = Observation(agent, trajectoryData, envMap, valid)
				observations[agent.agentId] = s
			# An AGC member
			else:
				# The FOV list contains field of view per timestamp
				# For each timestamp, it contains a list of individual FOVs per sensor on the vehicle
				# Each FOV per sensor has 2 lists: [x1, x2, x3, ...],[y1, y2, y3, ...]
				agents[member.agentId] = member
				for i in range(len(entity["FOV"])):
					# FIXME: For now there is only one sensor per vehicle, we might have to union them later
					xs = entity["FOV"][i][0][0]
					ys = entity["FOV"][i][0][1]
					coords = [[xs[j], ys[j]] for j in range(len(xs))]
					fov.append(coords, entity["Datap"][i][0], agentIndex=len(agents) - 1)
		return (observations, agents)

class SpecParser(Parser):
	def __init__(self, dirAbsPath):
		super().__init__(dirAbsPath)
		self._specsPath = os.path.abspath(os.path.join(self._dirAbsPath, "specs.json"))

	def parse(self):
		"""
		Returns
		===
		A dict `(observations, agents)`
		Stories are sensor readings, Agents are sensor locations
		"""
		with open(self._specsPath, 'r') as jsonFile:
			parsedObservation = json.load(jsonFile)
			i = 0
