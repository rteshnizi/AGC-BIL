from typing import Tuple
import json
import os

from bil.model.featureMap import FeatureMap
from bil.model.map import Map

class Parser(object):
	def __init__(self, dirAbsPath):
		self._dirAbsPath = dirAbsPath
		self._fmPath = os.path.abspath(os.path.join(self._dirAbsPath, "fm.json"))
		self._saMapPath = os.path.abspath(os.path.join(self._dirAbsPath, "sam.json"))
		self._ptListFile = os.path.abspath(os.path.join(self._dirAbsPath, "dt.json"))

	def parse(self) -> Tuple[object, object]:
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
