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
		self.renderArrows = False
		self._lineIds = None
		self._circleIds = []
		self._arrowIds = []

	def render(self, canvas, time=None):
		if self._lineIds is not None: return
		self._lineIds = []
		spawnColor = "GREEN"
		vanishedColor = "RED"
		linePoses = []
		for p in self.poses:
			linePoses.append(p)
			if p.spawn:
				fillColor = spawnColor
			elif p.vanished:
				fillColor = vanishedColor
				self._lineIds.append(Drawing.CreateLine(canvas, [[p.pt.x, p.pt.y] for p in linePoses], color=self._color, tag=self.name, width=3))
				linePoses = []
			else:
				fillColor = ""
			poseIsActive = p.time == time
			if time is None or poseIsActive:
				self._circleIds.append(Drawing.CreateCircle(canvas, p.x, p.y, radius=5, outline=self.MARKING_COLOR, tag="%s-%.1f" % (self.name, p.time), fill=fillColor))
			if not self.renderArrows: continue
			(dx, dy) = Geometry.getUnitVectorFromAngle(p.angleFromX)
			dx = dx * self.HEADING_ARROW_LENGTH
			dy = dy * self.HEADING_ARROW_LENGTH
			self._arrowIds.append(Drawing.CreateLine(canvas, [[p.pt.x, p.pt.y], [p.pt.x + dx, p.pt.y + dy]], color=self.MARKING_COLOR, tag=self.name, arrow=True))

	def clear(self, canvas):
		if self._lineIds is None: return
		for lineId in self._lineIds:
			Drawing.RemoveShape(canvas, lineId)
		for shapeId in self._circleIds + self._arrowIds:
			Drawing.RemoveShape(canvas, shapeId)
		self._lineIds = None
		self._circleIds = []
		self._arrowIds = []
