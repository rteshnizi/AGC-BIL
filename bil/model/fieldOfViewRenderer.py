from bil.gui.drawing import Drawing
from bil.model.connectivityGraph import ConnectivityGraph

class FieldOfViewRenderer:
	"""
	Used for debug rendering ONLY
	"""
	def __init__(self):
		self._previousCGraph: ConnectivityGraph = None
		self._timeLabelCanvasId = None

	def render(self, cGraph: ConnectivityGraph, canvas):
		if self._previousCGraph is not None and self._previousCGraph.time != cGraph.time: self.clearRender(canvas)
		for fov in cGraph.fovNodes:
			cGraph.nodes[fov]["region"].render(canvas)
		for shadowName in cGraph.shadowNodes:
			shadowRegion = cGraph.nodes[shadowName]["region"]
			shadowRegion.render(canvas)
		timeStr = "N/A" if cGraph is None else "%.2f" % cGraph.time
		labelStr = "t = %s" % timeStr
		self._timeLabelCanvasId = Drawing.CreateText(canvas, [-2, -5], labelStr, "TIME-LABEL")
		self._previousCGraph = cGraph

	def clearRender(self, canvas):
		if self._previousCGraph is None: return
		for fov in self._previousCGraph.fovNodes:
			self._previousCGraph.nodes[fov]["region"].clearRender(canvas)
		for shadowName in self._previousCGraph.shadowNodes:
			shadowRegion = self._previousCGraph.nodes[shadowName]["region"]
			shadowRegion.clearRender(canvas)
		Drawing.RemoveShape(canvas, self._timeLabelCanvasId)
		self._timeLabelCanvasId = None
		self._previousCGraph = None
