from bil.model.connectivityGraph import ConnectivityGraph

class FieldOfViewRenderer:
	"""
	Used for debug rendering ONLY
	"""
	def __init__(self):
		self._previousFov = None
		self._previousCGraph = None
		# Utility variable to cache the cGraphs
		self._cGraphCache = {}

	def render(self, envMap, fov, validators, canvas):
		if self._previousFov is not None and self._previousFov.time != fov.time: self.clearRender(canvas)
		for sensorId in fov.sensors:
			sensor = fov.sensors[sensorId]
			sensor.region.render(canvas)
		cGraph = None
		if fov.time not in self._cGraphCache:
			cGraph = ConnectivityGraph(envMap, fov, validators)
			self._cGraphCache[fov.time] = cGraph
		else:
			cGraph = self._cGraphCache[fov.time]
		for shadowName in cGraph.shadows:
			shadowRegion = cGraph.nodes[shadowName]["region"]
			shadowRegion.render(canvas)
		self._previousFov = fov
		self._previousCGraph = cGraph

	def clearRender(self, canvas):
		if self._previousFov is None: return
		for sensorId in self._previousFov.sensors:
			sensor = self._previousFov.sensors[sensorId]
			sensor.region.clearRender(canvas)
		for shadowName in self._previousCGraph.shadows:
			shadowRegion = self._previousCGraph.nodes[shadowName]["region"]
			shadowRegion.clearRender(canvas)
		self._previousFov = None
		self._previousCGraph = None
