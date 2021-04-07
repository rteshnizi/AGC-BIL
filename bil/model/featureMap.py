from typing import Dict

class FeatureMap:
	def __init__(self, json: dict):
		self.features: Dict[str, Feature] = {}
		for f in json:
			self.features[f["Name"]] = Feature(f["Name"], f["feature"])

class Feature:
	def __init__(self, name, raw: dict):
		self._raw = raw
		self.name = name
		# FIXME: There is a difference in capitalization in easy2cross vs easy2Cross
		self.easyToCross = EasyToCross(raw["easy2cross"]["Car"], raw["easy2Cross"]["pedestrian"], raw["easy2Cross"]["av"])
		self.seeThrough = SeeThrough(raw["seethrough"])

class EasyToCross:
	def __init__(self, car, pedestrian, av):
		self.car = car
		self.pedestrian = pedestrian
		self.av = av

class SeeThrough:
	def __init__(self, raw):
		self.fromAbove = raw["fromAbove"] == "yes"
		self.fromSide = raw["fromSide"] == "yes"
