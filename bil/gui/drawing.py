import random
from tkinter import LAST
from shapely.geometry import Point

TRANSLATION_X = 60
TRANSLATION_Y = 60
SCALE = 7.2

class Drawing:
	@staticmethod
	def _translateCoords(coord):
		c = [SCALE * (coord[0] + TRANSLATION_X), SCALE * ((-1 * coord[1]) + TRANSLATION_Y)]
		return c

	@staticmethod
	def RandomColorString():
		return "#"+''.join([random.choice('0123456789ABCDEF') for j in range(6)])

	@staticmethod
	def CreateCircle(canvas, centerX, centerY, radius, outline, tag, fill="", width=1):
		"""
		Returns shape id

		center: Point

		radius: number

		outline: color string (empty string for transparent)

		fill: color string (empty string for transparent)

		width: number

		tag: a unique identifier (use entity name)
		"""
		c = Drawing._translateCoords([centerX, centerY])
		centerX = c[0]
		centerY = c[1]
		topLeft = Point((centerX - radius, centerY - radius))
		bottomRight = Point((centerX + radius, centerY + radius))
		shape = canvas.create_oval((topLeft.x, topLeft.y, bottomRight.x, bottomRight.y), outline=outline, fill=fill, width=width, tag=tag)
		# bindMouseEvent(canvas, shape)
		return shape

	@staticmethod
	def CreatePolygon(canvas, coords, outline, fill, width, tag, hashFill=False, hashDensity=25):
		"""
		Returns shape id

		coords: A list of coordinate pairs [x, y]

		outline: color string (empty string for transparent)

		fill: color string (empty string for transparent)

		width: number

		tag: a unique identifier (use entity name)
		"""
		if hashDensity not in [75, 50, 25, 12]: raise AssertionError("Density should be one of 75, 50, 25, or 12.")
		coords = [Drawing._translateCoords(c) for c in coords]
		hashStr = "gray%d" % hashDensity if hashFill else ""
		shape = canvas.create_polygon(coords, outline=outline, fill=fill, width=width, tag=tag, stipple=hashStr)
		# Drawing.bindMouseEvent(canvas, shape)
		return shape

	@staticmethod
	def CreateLine(canvas, coords, color, tag, width=1, dash=(), arrow=False):
		"""
		Returns shape id, or None if there are no points.

		coords: A list of coordinate pairs [x, y]

		color: color string (empty string for transparent)

		width: number; default is 1

		dash: Dash pattern, given as a list of segment lengths. Only the odd segments are drawn.

		tag: a unique identifier (use entity name)
		"""
		if len(coords) == 0: return None
		coords = [Drawing._translateCoords(c) for c in coords]
		shape = canvas.create_line(coords, fill=color, width=width, dash=dash, tag=tag, arrow=LAST if arrow else None)
		# Drawing.bindMouseEvent(canvas, shape)
		return shape

	@staticmethod
	def CreateText(canvas, coords, text, tag, color="Black", fontSize=10):
		"""
		Returns shape id

		coords: A list of coordinate pairs [x, y]

		text: to be rendered

		color: color string (default black)

		fontSize: number; default is 10
		"""
		coords = Drawing._translateCoords(coords)
		shape = canvas.create_text(coords[0], coords[1], text=text, fill=color, font="Times %d" % fontSize, tag=tag)
		# Drawing.bindMouseEvent(canvas, shape)
		return shape

	# @staticmethod
	# def bindMouseEvent(canvas, shape):
	# 	canvas.tag_bind(shape, '<Enter>', Drawing.mouseHandler)

	# @staticmethod
	# def mouseHandler(event):
	# 	if not model.app.shouldPrintMouse: return
	# 	shape = event.widget.find_closest(event.x, event.y)
	# 	tag = model.canvas.tkCanvas.gettags(shape)[0]
	# 	entity = model.entities.get(tag)
	# 	if not entity: return
	# 	if hasattr(entity, 'loc'):
	# 		print('%s-%d,%d' % (tag, entity.loc.x(), entity.loc.y()))
	# 	else:
	# 		print(tag)

	@staticmethod
	def RemoveShape(canvas, shapeId):
		"""
		Remove a shape from canvas
		"""
		canvas.delete(shapeId)
