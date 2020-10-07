from shapely.geometry import LineString, Point
from itertools import compress

from bil.model.pose import Pose
from bil.gui.drawing import Drawing
from bil.utils.geometry import Geometry

class Trajectory:
	def __init__(self, name, coords):
		# [#time, #x, #y, #angleWithXAxis]
		self._coords = coords
		# self._color = Drawing.RandomColorString()
		self._color = "CornflowerBlue"
		self.poses = [Pose(c[0], c[1], c[2], c[3]) for c in coords]
		self.name = name
		self.line = LineString([p.pt for p in self.poses])
		self.MARKING_COLOR = "PURPLE"
		self.HEADING_ARROW_LENGTH = 5
		self.renderArrows = True

	def buildString(self, graph: "ConnectivityGraph"):
		s = []
		for p in self.poses:
			for r in graph._disjointPolys:
				if Geometry.isPointInsidePolygon(p.pt, r.polygon):
					# print("%.5f,%.5f in %s" % (p.pt.x, p.pt.y, r.name))
					if len(s) == 0 or s[len(s) - 1] != r.name: s.append(r.name)
					break
		return s

	def clip(self, valid) -> "Trajectory":
		# FIXME: this has to make a list of disjoint lists
		# Only tale the observed portions of the trajectory
		coords = list(compress(self._coords, valid))
		return Trajectory("%s-clipped" % self.name, coords)

	def render(self, canvas):
		Drawing.CreateLine(canvas, [[p.pt.x, p.pt.y] for p in self.poses], color=self._color, tag=self.name, width=3)
		for p in self.poses:
			Drawing.CreateCircle(canvas, p.x, p.y, radius=5, outline=self.MARKING_COLOR, tag="%s-%.1f" % (self.name, p.time))
			if not self.renderArrows: continue
			(dx, dy) = Geometry.getUnitVectorFromAngle(p.angleFromX)
			dx = dx * self.HEADING_ARROW_LENGTH
			dy = dy * self.HEADING_ARROW_LENGTH
			Drawing.CreateLine(canvas, [[p.pt.x, p.pt.y], [p.pt.x + dx, p.pt.y + dy]], color=self.MARKING_COLOR, tag=self.name, arrow=True)

	def validate(self, envMap):
		for pose in self.poses:
			for region in envMap.regions.values():
				if region.isObstacle and region.isInsideRegion(pose.x, pose.y):
					return False
		return True
