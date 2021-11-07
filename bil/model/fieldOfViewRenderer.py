from bil.model.connectivityGraph import ConnectivityGraph

class FieldOfViewRenderer:
	"""
	Used for debug rendering ONLY
	"""
	def __init__(self):
		self._previousCGraph: ConnectivityGraph = None

	def render(self, cGraph: ConnectivityGraph, canvas):
		if self._previousCGraph is not None and self._previousCGraph.timestamp != cGraph.timestamp: self.clearRender(canvas)
		for fov in cGraph.fovNodes:
			cGraph.nodes[fov]["region"].render(canvas)
		for shadowName in cGraph.shadowNodes:
			shadowRegion = cGraph.nodes[shadowName]["region"]
			shadowRegion.render(canvas)
		self._previousCGraph = cGraph

	def clearRender(self, canvas):
		if self._previousCGraph is None: return
		for fov in self._previousCGraph.fovNodes:
			self._previousCGraph.nodes[fov]["region"].clearRender(canvas)
		for shadowName in self._previousCGraph.shadowNodes:
			shadowRegion = self._previousCGraph.nodes[shadowName]["region"]
			shadowRegion.clearRender(canvas)
		self._previousCGraph = None
