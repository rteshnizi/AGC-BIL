import json
import os
from typing import Dict

from bil.model.agent import Agent
from bil.model.teamMember import TeamMember
from bil.model.map import Map
from bil.model.fieldOfView import FieldOfView
from bil.model.featureMap import FeatureMap
from bil.model.story import Story

class Scenario(object):
	def __init__(self, featureMap, envMap, obsPath=None):
		"""
		if `obsPath == None`, then it is assumed that live updates are going to be added
		"""
		self._obsPath = obsPath
		self._parsedObservation: dict = None
		# Only AGC team members, does not include observed agents
		# Agent ID -> Agent
		self.agents: Dict[int, Story] = {}
		# Agent ID -> Story
		self.stories: Dict[int, Story] = {}
		self.featureMap: FeatureMap = None
		# FieldOfView is assigned after map is built
		self.fov: FieldOfView = featureMap
		self.map: Map = envMap
		self._build()

	def _build(self):
		self.fov = FieldOfView(self.map)
		if self._obsPath is not None:
			self._buildObservations()

	def _buildObservations(self) -> None:
		with open(self._obsPath, 'r') as jsonFile:
			self._parsedObservation = json.load(jsonFile)
		idNum = 0
		for entity in self._parsedObservation:
			idNum += 1
			# Not a team member
			if "valid" in entity:
				agent = Agent(idNum)
				# self.agents[agent.agentId] = agent
				trajectoryData = entity["Datap"]
				valid = entity["valid"]
				s = Story(agent, trajectoryData, self.map, valid)
				self.stories[agent.agentId] = s
			# An AGC member
			else:
				# The FOV list contains field of view per timestamp
				# For each timestamp, it contains a list of individual FOVs per sensor on the vehicle
				# Each FOV per sensor has 2 lists: [x1, x2, x3, ...],[y1, y2, y3, ...]
				member = TeamMember(idNum)
				self.agents[member.agentId] = member
				for i in range(len(entity["FOV"])):
					# FIXME: For now there is only one sensor per vehicle, we might have to union them later
					xs = entity["FOV"][i][0][0]
					ys = entity["FOV"][i][0][1]
					coords = [[xs[j], ys[j]] for j in range(len(xs))]
					self.fov.append(coords, entity["Datap"][i][0], agentIndex=len(self.agents) - 1)

	def validate(self):
		spec = ["r38-0", "r72-0"]
		graph = self.fov.cGraphs[0]
		s: Story = None
		for s in self.stories.values():
			isValid = s.trajectory.validate(self.map)
			if not isValid:
				print("Trajectory of agent %s is invalid" % s.agent.name)
				return
			print("validating specification %s against agent %s" % (repr(spec), s.agent.name))
			# isValid = s.validate(self.map, self.fov, verbose=False)
			isValid = s.validateWithSpecification(self.map, self.fov, graph, spec, s.getSensorReadings(graph))
			print("The specification %s is %s" % (repr(spec), "valid" if isValid else "invalid"))
