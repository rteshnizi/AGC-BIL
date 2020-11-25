from abc import ABC
from typing import Tuple
import json
import os

from bil.model.featureMap import FeatureMap
from bil.model.map import Map
from bil.spec.specification import Specification
from bil.model.observationOld import ObservationOld
from bil.model.observations import Observations
from bil.model.agent import Agent
from bil.model.teamMember import TeamMember

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
		self._obsTianqiJsonPath = os.path.abspath(os.path.join(self._dirAbsPath, "obs-tianqi.json"))
		self._agentIdKey = "AgentID"
		self._trackIdKey = "trackID"
		self._pointsKey = "Datap"
		self._fovKey = "FoV"

	def _parse(self, envMap, fov):
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
				agent = Agent(idNum)
				trajectoryData = entity["Datap"]
				valid = entity["valid"]
				s = ObservationOld(agent, trajectoryData, envMap, valid)
				observations[agent.agentId] = s
			# An AGC member
			else:
				# The FOV list contains field of view per timestamp
				# For each timestamp, it contains a list of individual FOVs per sensor on the vehicle
				# Each FOV per sensor has 2 lists: [x1, x2, x3, ...],[y1, y2, y3, ...]
				member = TeamMember(idNum)
				agents[member.agentId] = member
				for i in range(len(entity["FOV"])):
					# FIXME: For now there is only one sensor per vehicle, we might have to union them later
					xs = entity["FOV"][i][0][0]
					ys = entity["FOV"][i][0][1]
					coords = [[xs[j], ys[j]] for j in range(len(xs))]
					fov.append(coords, entity["Datap"][i][0], agentIndex=len(agents) - 1)
		return (observations, agents)

	def parse(self, envMap, fov):
		"""
		Returns
		===
		A tuple `(observations, sens)`
		Stories are sensor readings, Agents are sensor locations
		"""
		with open(self._observationsJsonPath, 'r') as jsonFile:
			parsedObservation = json.load(jsonFile)
		agentNum = 0
		idNum = 0
		observations = Observations()
		for entity in parsedObservation:
			idNum += 1
			# Not a team member
			if "valid" in entity:
				trajectoryData = entity["Datap"]
				valid = entity["valid"]
				for i in range(len(trajectoryData)):
					if valid[i] == 1:
						observations.appendTrack(idNum, trajectoryData[i])
			# An AGC member
			else:
				# FIXME: right now it's duplicated in _parse which is also beng called
				pass
		observations.mapFov(fov)
		return observations

	def parseNew(self, envMap, fov):
		"""
		Returns
		===
		A tuple `observations`
		Stories are sensor readings, Agents are sensor locations
		"""
		with open(self._obsTianqiJsonPath, 'r') as jsonFile:
			parsedObservation = json.load(jsonFile)
		observations = {}
		for entity in parsedObservation:
			# Not a team member
			if self._trackIdKey in entity:
				trackId = entity[self._trackIdKey]
				trajectoryData = entity[self._pointsKey]
				obs = ObservationOld(trackId, trajectoryData, envMap)
				observations[trackId] = obs
			# An AGC member
			else:
				agentId = entity[self._agentIdKey]
				# The FOV list contains field of view per timestamp
				# For each timestamp, it contains a list of individual FOVs per sensor on the vehicle
				# Each FOV per sensor has 2 lists: [x1, x2, x3, ...],[y1, y2, y3, ...]
				for i in range(len(entity[self._fovKey])):
					# FIXME: For now there is only one sensor per vehicle, we might have to union them later
					xs = entity[self._fovKey][i][0][0]
					ys = entity[self._fovKey][i][0][1]
					coords = [[xs[j], ys[j]] for j in range(len(xs))]
					fov.append(coords, entity[self._pointsKey][i][0], agentIndex=agentId)
		return observations

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
		specs = []
		with open(self._specsPath, 'r') as jsonFile:
			specsJson = json.load(jsonFile)
		for specName in specsJson:
			specs.append(Specification(specName, specsJson[specName]))
		return specs
