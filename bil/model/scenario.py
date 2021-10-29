from bil.model.map import Map
from bil.model.featureMap import FeatureMap

class Scenario(object):
	def __init__(self, featureMap, envMap, observations):
		self.featureMap: FeatureMap = featureMap
		self.map: Map = envMap
		self.observations = observations
