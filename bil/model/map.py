from typing import Dict
from shapely.geometry import Polygon
from bil.model.featureMap import FeatureMap
from bil.model.mapRegion import MapRegion
from bil.utils.geometry import Geometry

class Map:
	def __init__(self, coords, regionVertIndices, typesPerRegion, featureMap):
		self.coords = coords
		self.regions: Dict[str, MapRegion] = {}
		self._populateRegions(regionVertIndices, typesPerRegion, featureMap)
		self.polygon: Polygon = self._buildPolygon()

	def _populateRegions(self, regionVertIndices, typesPerRegion, featureMap: FeatureMap):
		for i in range(len(regionVertIndices)):
			region = regionVertIndices[i]
			if len(region) == 2: continue
			# FIXME: If data is coming from matlab then the indices there are 1-based
			coords = [self.coords[ind] for ind in region]
			regionType = typesPerRegion[i]
			name = "r%d" % i
			self.regions[name] = MapRegion(name, coords, regionType, featureMap.features[regionType])

	def _buildPolygon(self):
		polygons = [self.regions[r].polygon for r in self.regions]
		polygon = Geometry.union(polygons)
		return polygon

	def render(self, canvas):
		for region in self.regions.values():
			region.render(canvas)

	def clearRender(self, canvas):
		for region in self.regions.values():
			region.clearRender(canvas)
