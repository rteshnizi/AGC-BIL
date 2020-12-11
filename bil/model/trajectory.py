from shapely.geometry import LineString, Point
from itertools import compress

from bil.observation.pose import Pose
from bil.gui.drawing import Drawing
from bil.utils.geometry import Geometry

class Trajectory:
	def __init__(self, name, poses):
		# [#time, #x, #y, #angleWithXAxis]
		# self._color = Drawing.RandomColorString()
		self.name = name
		self._color = "CornflowerBlue"
		self.poses = poses
		self.line = LineString([p.pt for p in self.poses])
		self.MARKING_COLOR = "PURPLE"
		self.HEADING_ARROW_LENGTH = 5
		self.renderArrows = True
		self._lineId = None
		self._circleIds = []
		self._arrowIds = []

	def render(self, canvas):
		if self._lineId is not None: return
		self._lineId = Drawing.CreateLine(canvas, [[p.pt.x, p.pt.y] for p in self.poses], color=self._color, tag=self.name, width=3)
		for p in self.poses:
			self._circleIds.append(Drawing.CreateCircle(canvas, p.x, p.y, radius=5, outline=self.MARKING_COLOR, tag="%s-%.1f" % (self.name, p.time)))
			if not self.renderArrows: continue
			(dx, dy) = Geometry.getUnitVectorFromAngle(p.angleFromX)
			dx = dx * self.HEADING_ARROW_LENGTH
			dy = dy * self.HEADING_ARROW_LENGTH
			self._arrowIds.append(Drawing.CreateLine(canvas, [[p.pt.x, p.pt.y], [p.pt.x + dx, p.pt.y + dy]], color=self.MARKING_COLOR, tag=self.name, arrow=True))

	def clear(self, canvas):
		if self._lineId is None: return
		Drawing.RemoveShape(canvas, self._lineId)
		for shapeId in self._circleIds + self._arrowIds:
			Drawing.RemoveShape(canvas, shapeId)
		self._lineId = None
		self._circleIds = []
		self._arrowIds = []
