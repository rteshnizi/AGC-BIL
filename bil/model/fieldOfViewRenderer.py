from bil.gui.drawing import Drawing
from bil.model.connectivityGraph import ConnectivityGraph
from bil.model.sensingRegion import SensingRegion
from bil.observation.track import Tracks

class FieldOfViewRenderer:
	"""
	Used for debug rendering ONLY
	"""
	def __init__(self):
		self._previousCGraph: ConnectivityGraph = None
		self._timeLabelCanvasId = None

	def render(self, cGraph: ConnectivityGraph, canvas):
		if self._previousCGraph is not None and self._previousCGraph.time != cGraph.time: self.clearRender(canvas)
		for fovName in cGraph.fovNodes:
			sensingRegion: SensingRegion = cGraph.nodes[fovName]["region"]
			sensingRegion.render(canvas)
			for trackId in sensingRegion.tracks:
				sensingRegion.tracks[trackId].render(canvas)
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
			fovNodeData = self._previousCGraph.nodes[fov]
			fovNodeData["region"].clearRender(canvas)
			for trackId in fovNodeData["tracks"]:
				fovNodeData["tracks"][trackId].clearRender(canvas)
		for shadowName in self._previousCGraph.shadowNodes:
			shadowRegion = self._previousCGraph.nodes[shadowName]["region"]
			shadowRegion.clearRender(canvas)
		Drawing.RemoveShape(canvas, self._timeLabelCanvasId)
		self._timeLabelCanvasId = None
		self._previousCGraph = None
