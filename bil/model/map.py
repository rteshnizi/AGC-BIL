from typing import Dict
from bil.model.featureMap import FeatureMap
from bil.model.mapRegion import MapRegion

class Map:
	def __init__(self, coords, regionVertIndices, typesPerRegion, featureMap):
		self.coords = coords
		self.regions: Dict[str, MapRegion] = {}
		self._buildRegions(regionVertIndices, typesPerRegion, featureMap)

	def _buildRegions(self, regionVertIndices, typesPerRegion, featureMap: FeatureMap):
		for i in range(len(regionVertIndices)):
			region = regionVertIndices[i]
			if len(region) == 2: continue
			# FIXME: Data is coming from matlab and the indices there are 1-based
			coords = [self.coords[ind - 1] for ind in region]
			regionType = typesPerRegion[i]
			name = "r%d" % i
			self.regions[name] = MapRegion(name, coords, regionType, featureMap.features[regionType])

	def render(self, canvas):
		for region in self.regions.values():
			region.render(canvas)

	def clearRender(self, canvas):
		for region in self.regions.values():
			region.clearRender(canvas)
