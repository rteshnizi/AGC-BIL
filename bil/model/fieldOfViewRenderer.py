from bil.observation.fov import FOV
from bil.utils.graph import GraphAlgorithms
from bil.model.sensingRegion import SensingRegion
from bil.model.connectivityGraph import ConnectivityGraph
from bil.model.timedGraph import TimedGraph

class FieldOfViewRenderer:
	"""
	Used for debug rendering ONLY
	"""
	def __init__(self):
		self._previousFov = None
		self._previousCGraph = None
		# Utility variable to cache the cGraphs
		self._cGraphCache = {}

	# def chainedGraphThroughTime(self, spec):
	# 	if self._chainedGraphThroughTime is None:
	# 		self._chainedGraphThroughTime = TimedGraph(self.cGraphs, spec)
	# 	return self._chainedGraphThroughTime

	def render(self, envMap, fov, canvas):
		if self._previousFov is not None and self._previousFov.time != fov.time: self.clearRender(canvas)
		for sensorId in fov.sensors:
			sensor = fov.sensors[sensorId]
			sensor.region.render(canvas)
		cGraph = None
		if fov.time not in self._cGraphCache:
			cGraph = ConnectivityGraph(envMap, fov)
			self._cGraphCache[fov.time] = cGraph
		else:
			cGraph = self._cGraphCache[fov.time]
		for p in cGraph._disjointPolys:
			p.render(canvas)
		self._previousFov = fov
		self._previousCGraph = cGraph

	def clearRender(self, canvas):
		if self._previousFov is None: return
		for sensorId in self._previousFov.sensors:
			sensor = self._previousFov.sensors[sensorId]
			sensor.region.clearRender(canvas)
		for p in self._previousCGraph._disjointPolys:
			p.clearRender(canvas)
		self._previousFov = None
		self._previousCGraph = None
