from typing import Dict

from bil.observation.sensor import Sensor
from bil.utils.geometry import Geometry

class Fov:
	def __init__(self, sensors):
		self._polygon = None
		self.sensors: Dict[tuple, Sensor] = sensors

	def __repr__(self):
		return "FOV-%s" % repr(self.sensors)

	@property
	def polygon(self):
		if self._polygon is None:
			polygons = [self.sensors[i].region.polygon for i in self.sensors]
			self._polygon = Geometry.union(polygons)
		return self._polygon

	@property
	def time(self):
		if len(self.sensors) == 0: return None
		for sensorId in self.sensors:
			return self.sensors[sensorId].pose.time

	def getEquivalentSensorById(self, sensorId: tuple) -> Sensor:
		return self.sensors[(self.time, sensorId[1])]
