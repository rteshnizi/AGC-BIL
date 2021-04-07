from shapely.geometry import Polygon
from bil.model.featureMap import FeatureMap
from bil.model.map import Map
from bil.model.sensingRegion import SensingRegion
from bil.observation.pose import Pose
from bil.utils.geometry import Geometry

class Sensor:
	def __init__(self, idNum: int, time: float, x: float, y: float, psi: float, coords: Geometry.CoordsList, envMap: Map, featureMap: FeatureMap):
		self.id = idNum
		self.pose = Pose(time, x, y, psi)
		self._originalCoords = [tuple(coord) for coord in coords]
		poly = self._buildVisibilityPolygon(envMap, featureMap)
		self.region = SensingRegion("SR%d" % idNum, [], time, idNum, polygon=poly)

	def __repr__(self):
		return "Sensor%d" % self.id

	def _buildVisibilityPolygon(self, envMap: Map, featureMap: FeatureMap) -> Geometry.CoordsList:
		sortedCoords = Geometry.sortCoordinatesClockwise(self._originalCoords)
		polygon = Polygon(sortedCoords)
		for rKey in envMap.regions:
			region = envMap.regions[rKey]
			# FIXME: Currently, the type of sensor is missing.
			# Once its available you should check the type and see which type of seeThrough I should look for
			if featureMap.features[region.type].seeThrough.fromAbove: continue
			nextPolygon = Geometry.subtract(polygon, region.polygon)
			polygon = nextPolygon
		return polygon
