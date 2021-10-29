from bil.utils.geometry import Geometry
import tkinter as tk

from bil.gui.canvas import Canvas
from bil.model.timedGraph import TimedGraph
from bil.model.fieldOfViewRenderer import FieldOfViewRenderer
from bil.model.connectivityGraph import ConnectivityGraph
from bil.utils.graph import GraphAlgorithms
from bil.gui.drawing import Drawing

class App(tk.Frame):
	def __init__(self, bil, validateCallback):
		self.bil = bil
		# This is the method to call when validate button is clicked
		self._validateCallback = validateCallback
		self._maxFovIndex = len(self.bil.observations) - 1
		self.master = tk.Tk()
		self.fovRenderer = FieldOfViewRenderer()
		self.master.title("Canvas")
		self.master.geometry("1000x900")
		self.master.protocol("WM_DELETE_WINDOW", self._onClose)

		# Create & Configure frame
		tk.Grid.rowconfigure(self.master, 0, weight=1)
		tk.Grid.columnconfigure(self.master, 0, weight=1)
		self.frame = tk.Frame(self.master)
		self.frame.grid(row=0, column=0, sticky=tk.N + tk.S + tk.E + tk.W)
		tk.Grid.rowconfigure(self.frame, 0, minsize=25)
		tk.Grid.rowconfigure(self.frame, 1, minsize=25)
		tk.Grid.rowconfigure(self.frame, 2, minsize=850)
		super().__init__(self.master)
		self.fovLabel = tk.StringVar(master=self.master)
		self._fovIndex = 0
		self.eventLabel = tk.StringVar(master=self.master)
		self._eventIndex = 0
		self._eventDrawingId = []
		self.fovLabel.set(self._fovLabelText)
		self.validationBtnLabel = tk.StringVar(master=self.master)
		self._validationIndex = 0
		self.validationBtnLabel.set(self._validationBtnLabelText)
		self.specDropdownValue = tk.StringVar(self.master)
		self.specDropdownValue.set("simple-1") # set the default option
		self._dbg = {
			"Render Trajectory": tk.IntVar(master=self.master, value=1),
			"Display Geom Graph": tk.IntVar(master=self.master, value=0),
			"Display Spring Graph": tk.IntVar(master=self.master, value=1),
			"Show FOV": tk.IntVar(master=self.master, value=0),
			"Show Event": tk.IntVar(master=self.master, value=0),
		}
		self.createButtons()
		self.createDebugOptions()
		self.chained = None
		self.canvas = Canvas(master=self.frame, app=self, row=2, col=0)
		self._renderMap()
		self._renderTrajectories()
		self._renderSpec()

	def _renderMap(self):
		self.bil.map.render(self.canvas.tkCanvas)
		# Draw Axis
		Drawing.CreateLine(self.canvas.tkCanvas, [[0, 0], [5, 0]], "PURPLE", "X-AXIS", width=2, arrow=True)
		Drawing.CreateText(self.canvas.tkCanvas, [5, -5], "X", "X-AXIS-L")
		Drawing.CreateLine(self.canvas.tkCanvas, [[0, 0], [0, 5]], "PURPLE", "Y-AXIS", width=2, arrow=True)
		Drawing.CreateText(self.canvas.tkCanvas, [-5, 5], "Y", "Y-AXIS-L")

	def _renderTrajectories(self, force=False):
		if not force and not self.shouldRenderTrajectory: return
		trajectories = self.bil.observations.trajectories
		t = self.bil.observations.timeArray[self._fovIndex] if self.shouldShowFOV else None
		for trajectory in trajectories:
			trajectory.render(self.canvas.tkCanvas, time=t)

	def _clearTrajectories(self, force=False):
		if not force and self.shouldRenderTrajectory: return
		trajectories = self.bil.observations.trajectories
		for trajectory in trajectories:
			trajectory.clear(self.canvas.tkCanvas)

	def _clearSpec(self):
		self.spec.nfa.killDisplayedGraph()
		self.spec.clearRender(self.canvas.tkCanvas)

	def _renderSpec(self):
		self.spec.render(self.canvas.tkCanvas)

	@property
	def _validationBtnLabelText(self):
		return "Validate T = %.2f" % self.bil.observations.timeArray[self._validationIndex]

	@property
	def _fovLabelText(self):
		return "0 ≤ FOV %d ≤ %d" % (self._fovIndex, self._maxFovIndex)

	def _onClose(self):
		GraphAlgorithms.killAllDisplayedGraph()
		self.master.destroy()

	def _toggleTrajectory(self, force=False):
		self._clearTrajectories(force)
		self._renderTrajectories(force)

	@property
	def _maxEventIndex(self):
		if self.chained is None: return 0
		l = 0
		for eventCandidates in self.chained.eventsByLayer:
			l += len(eventCandidates)
		return l

	@property
	def _eventLabelText(self):
		if self.chained is None: return "N/A"
		return "0 ≤ Ev %d ≤ %d" % (self._eventIndex, self._maxEventIndex)

	def _toggleEvents(self):
		self._clearEvents()
		self._renderEvents()

	def _clearEvents(self, force=False):
		if self.chained is None: return
		if len(self._eventDrawingId) == 0: return
		if not force and self._dbg["Show Event"].get() == 1: return
		[Drawing.RemoveShape(self.canvas.tkCanvas, id) for id in self._eventDrawingId]
		self._eventDrawingId = []

	def _renderEvents(self):
		if self.chained is None: return
		self.eventLabel.set(self._eventLabelText)
		color = "RED" if self.chained.eventCandidates[self._eventIndex][2] == "ingoing" else "BLUE"
		self._eventDrawingId = [
			Drawing.CreatePolygon(self.canvas.tkCanvas, self.chained.eventCandidates[self._eventIndex][0].exterior.coords, color, "", 1, color),
			Drawing.CreateLine(self.canvas.tkCanvas, self.chained.eventCandidates[self._eventIndex][3].coords, "PURPLE", "", 2)
		]

	def _changeEvent(self, showNext: bool):
		self._clearEvents(force=True)
		self._eventIndex += 1 if showNext else -1
		self._eventIndex = min(self._maxEventIndex, self._eventIndex)
		self._eventIndex = max(0, self._eventIndex)
		self._renderEvents()

	def _nextEvent(self):
		self._changeEvent(True)

	def _prevEvent(self):
		self._changeEvent(False)

	def _toggleFov(self):
		self._clearFov()
		self._toggleTrajectory(force=True)
		self._renderFov()

	def _clearFov(self):
		self.fovRenderer.clearRender(self.canvas.tkCanvas)

	def _renderFov(self):
		if not self.shouldShowFOV: return
		self.fovRenderer.render(self.bil.map, self.observationToRender.fov, self.spec.validators, self.canvas.tkCanvas)

	def _changeFov(self, showNext: bool):
		self._clearFov()
		self._fovIndex += 1 if showNext else -1
		self._fovIndex = min(self._maxFovIndex, self._fovIndex)
		self._fovIndex = max(0, self._fovIndex)
		self.fovLabel.set(self._fovLabelText)
		self._toggleTrajectory(force=True)
		self._renderFov()

	def _nextFov(self):
		self._changeFov(True)

	def _prevFov(self):
		self._changeFov(False)

	def _onSpecChange(self, event):
		self._clearSpec()
		self._renderSpec()

	@property
	def shouldRenderTrajectory(self) -> bool:
		return self._dbg["Render Trajectory"].get() == 1

	@property
	def shouldShowFOV(self) -> bool:
		return self._dbg["Show FOV"].get() == 1

	@property
	def displayGeomGraph(self) -> bool:
		return self._dbg["Display Geom Graph"].get() == 1

	@property
	def displaySpringGraph(self) -> bool:
		return self._dbg["Display Spring Graph"].get() == 1

	@property
	def spec(self):
		return next(spec for spec in self.bil.specs if spec.name == self.specDropdownValue.get())

	@property
	def observationToValidate(self):
		return self.bil.observations.getObservationByIndex(self._validationIndex)

	@property
	def observationToRender(self):
		return self.bil.observations.getObservationByIndex(self._fovIndex)

	def showSpecGraph(self):
		self.spec.nfa.killDisplayedGraph()
		self.spec.nfa.displayGraph()

	def showGraph(self):
		graph = ConnectivityGraph(self.bil.map, self.observationToRender.fov, self.spec.validators)
		graph.displayGraph(displayGeomGraph=self.displayGeomGraph, displaySpringGraph=self.displaySpringGraph)

	def chainGraphs(self):
		if self._fovIndex == self._maxFovIndex:
			previousIndex = self._fovIndex - 1
			nextIndex = self._fovIndex
		else:
			previousIndex = self._fovIndex
			nextIndex = self._fovIndex + 1
		g1 = ConnectivityGraph(self.bil.map, self.bil.observations.getObservationByIndex(previousIndex).fov, self.spec.validators)
		g2 = ConnectivityGraph(self.bil.map, self.bil.observations.getObservationByIndex(nextIndex).fov, self.spec.validators)
		self.chained = TimedGraph([g1, g2])
		# [Drawing.CreatePolygon(self.canvas.tkCanvas, p.exterior.coords, "RED", "", 1, "RED") for p in self.chained.red]
		# [Drawing.CreateLine(self.canvas.tkCanvas, l.coords, "PURPLE", "", 2) for l in self.chained.drawLine]

	def chainAll(self):
		GraphAlgorithms.displayGraphAuto(self.bil.fieldOfView.chainedGraphThroughTime(self.spec), displayGeomGraph=self.displayGeomGraph, displaySpringGraph=self.displaySpringGraph)

	def _createButton(self, row, col, text, callback, textVariable=None):
		tk.Grid.columnconfigure(self.frame, col, weight=1)
		btn = None
		if textVariable is not None:
			btn = tk.Button(self.frame, textvariable=textVariable)
		else:
			btn = tk.Button(self.frame)
			btn["text"] = text
		btn["command"] = callback
		btn.grid(row=row, column=col, sticky=tk.N + tk.S + tk.E + tk.W)

	def createButtons(self):
		row = 0
		column = 0

		self._createButton(row, column, "< FOV", self._prevFov)
		column += 1
		self._createButton(row, column, "FOV >", self._nextFov)
		column += 1

		self._createButton(row, column, "Dispaly G", self.showGraph)
		column += 1

		self._createButton(row, column, "Chain 1", self.chainGraphs)
		column += 1

		self._createButton(row, column, "Chain All", self.chainAll)
		column += 1

		self._createButton(row, column, "Validate", self.validate, self.validationBtnLabel)
		column += 1

		self._createButton(row, column, "Spec Graph", self.showSpecGraph)
		column += 1

		self._createButton(row, column, "< Event", self._prevEvent)
		column += 1
		self._createButton(row, column, "Event >", self._nextEvent)
		column += 1

	def createDebugOptions(self):
		tk.Grid.columnconfigure(self.frame, 0, weight=1)
		label = tk.Label(master=self.frame, textvariable=self.fovLabel)
		label.grid(row=1, column=0, columnspan=2)

		specList = [spec.name for spec in self.bil.specs]
		popupMenu = tk.OptionMenu(self.frame, self.specDropdownValue, *specList, command=self._onSpecChange)
		popupMenu.grid(row=1, column=2)

		column = 3
		for (text, variable) in self._dbg.items():
			tk.Grid.columnconfigure(self.frame, column, weight=1)
			checkbox = tk.Checkbutton(master=self.frame, text=text, variable=variable)
			if text == "Show FOV":
				checkbox["command"] = self._toggleFov
				checkbox.grid(row=1, column=column)
			elif text == "Render Trajectory":
				checkbox["command"] = self._toggleTrajectory
				checkbox.grid(row=1, column=column)
			elif text == "Show Event":
				checkbox["command"] = self._toggleEvents
				checkbox.grid(row=1, column=column)
			else:
				checkbox.grid(row=1, column=column)
			column += 1

		tk.Grid.columnconfigure(self.frame, 0, weight=1)
		label = tk.Label(master=self.frame, textvariable=self.eventLabel)
		label.grid(row=1, column=column)
		column += 1


	def condenseGraph(self):
		graph = ConnectivityGraph(self.bil.map, self.observationToRender.fov, self.spec.validators)
		condensed = graph.condense()
		GraphAlgorithms.displayGraphAuto(condensed, displayGeomGraph=self.displayGeomGraph, displaySpringGraph=self.displaySpringGraph)

	def validate(self):
		self._clearFov()
		self._dbg["Show FOV"].set(1)
		self._fovIndex = self._validationIndex - 1
		self._nextFov()
		self._validateCallback(self.observationToValidate)
		self._validationIndex += 1
		self.validationBtnLabel.set(self._validationBtnLabelText)
