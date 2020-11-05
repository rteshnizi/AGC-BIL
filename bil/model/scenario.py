import json
import os
from typing import Dict

from bil.model.map import Map
from bil.model.fieldOfView import FieldOfView
from bil.model.featureMap import FeatureMap
from bil.model.observation import Observation

class Scenario(object):
	def __init__(self, featureMap, envMap):
		self._parsedObservation: dict = None
		# Only AGC team members, does not include observed agents
		# Agent ID -> Agent
		self.agents: Dict[int, Observation] = {}
		# Agent ID -> Observation
		self.observations: Dict[int, Observation] = {}
		self.featureMap: FeatureMap = None
		# FieldOfView is assigned after map is built
		self.fov: FieldOfView = featureMap
		self.map: Map = envMap
		self._build()

	def _build(self):
		self.fov = FieldOfView(self.map)
