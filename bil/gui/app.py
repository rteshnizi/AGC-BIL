import os
import tkinter as tk
import time
from tkinter import filedialog

from bil.gui.canvas import Canvas
from bil.model.timedGraph import TimedGraph
from bil.utils.graph import GraphAlgorithms

class App(tk.Frame):
	def __init__(self, scenario, validateCallback, spec):
		self.scenario = scenario
		self.spec = spec
		# This is the method to call when validate button is clicked
		self._validateCallback = validateCallback
		self.timeSteps = len(self.scenario.fov) - 1
		self.master = tk.Tk()
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
		self.fovLabel.set(self._fovLabel)
		self._dbg = {
			"Print Mouse": tk.IntVar(master=self.master, value=0),
			"Render Trajectory": tk.IntVar(master=self.master, value=1),
			"Display Geom Graph": tk.IntVar(master=self.master, value=0),
			"Display Spring Graph": tk.IntVar(master=self.master, value=1),
			"Show FOV": tk.IntVar(master=self.master, value=0),
		}
		self.createButtons()
		self.createDebugOptions()
		self.canvas = Canvas(master=self.frame, app=self, row=2, col=0)
		self._renderScenario()
		self._renderTrajectories()
		self.lastDisplayedGraph = None

	def _renderScenario(self):
		self.scenario.map.render(self.canvas.tkCanvas)

	def _renderTrajectories(self):
		if not self.shouldRenderTrajectory: return
		for storyName in self.scenario.observations:
			self.scenario.observations[storyName].render(self.canvas.tkCanvas)

	def _clearTrajectories(self):
		if self.shouldRenderTrajectory: return
		for storyName in self.scenario.observations:
			self.scenario.observations[storyName].clear(self.canvas.tkCanvas)

	@property
	def _fovLabel(self):
		return "0 ≤ FOV %d ≤ %d" % (self._fovIndex, self.timeSteps)

	def _onClose(self):
		GraphAlgorithms.killAllDisplayedGraph()
		self.master.destroy()

	def _toggleTrajectory(self):
		self._clearTrajectories()
		self._renderTrajectories()

	def _toggleFov(self):
		self._clearFOV()
		self._renderFOV()

	def _clearFOV(self):
		self.scenario.fov.clearRender(self.canvas.tkCanvas)
		for p in self.scenario.fov.cGraphs[self._fovIndex]._disjointPolys:
			p.clearRender(self.canvas.tkCanvas)

	def _renderFOV(self):
		if not self.shouldShowFOV: return
		for region in self.scenario.fov[self._fovIndex]:
			region.render(self.canvas.tkCanvas)
		for p in self.scenario.fov.cGraphs[self._fovIndex]._disjointPolys:
			p.render(self.canvas.tkCanvas)

	@property
	def shouldPrintMouse(self) -> bool:
		return self._dbg["Print Mouse"].get() == 1

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

	def showGraph(self):
		self.lastDisplayedGraph = self.scenario.fov.cGraphs[self._fovIndex]
		self.lastDisplayedGraph.displayGraph(displayGeomGraph=self.displayGeomGraph, displaySpringGraph=self.displaySpringGraph)

	def _changeFov(self, showNext: bool):
		self._clearFOV()
		self._fovIndex += 1 if showNext else -1
		self._fovIndex = min(self.timeSteps, self._fovIndex)
		self._fovIndex = max(0, self._fovIndex)
		self.fovLabel.set(self._fovLabel)
		self._renderFOV()

	def nextFOV(self):
		self._changeFov(True)

	def prevFOV(self):
		self._changeFov(False)

	def chainGraphs(self):
		chained = TimedGraph(self.scenario.fov.cGraphs, self.spec, self._fovIndex, self._fovIndex + 2)
		GraphAlgorithms.displayGraphAuto(chained, displayGeomGraph=self.displayGeomGraph, displaySpringGraph=self.displaySpringGraph)

	def chainAll(self):
		GraphAlgorithms.displayGraphAuto(self.scenario.fov.chainedGraphThroughTime(self.spec), displayGeomGraph=self.displayGeomGraph, displaySpringGraph=self.displaySpringGraph)

	def _createButton(self, row, col, text, callback):
		tk.Grid.columnconfigure(self.frame, col, weight=1)
		btn = tk.Button(self.frame)
		btn["text"] = text
		btn["command"] = callback
		btn.grid(row=row, column=col, sticky=tk.N + tk.S + tk.E + tk.W)

	def createButtons(self):
		row = 0
		column = 0

		self._createButton(row, column, "Next FOV", self.nextFOV)
		column += 1

		self._createButton(row, column, "Prev FOV", self.prevFOV)
		column += 1

		self._createButton(row, column, "Dispaly G", self.showGraph)
		column += 1

		self._createButton(row, column, "Condense", self.condenseGraph)
		column += 1

		self._createButton(row, column, "Chain 1", self.chainGraphs)
		column += 1

		self._createButton(row, column, "Chain All", self.chainAll)
		column += 1

		self._createButton(row, column, "Validate", self.validate)
		column += 1

	def createDebugOptions(self):
		tk.Grid.columnconfigure(self.frame, 0, weight=1)
		label = tk.Label(master=self.frame, textvariable=self.fovLabel)
		label.grid(row=1, column=0, columnspan=2)

		column = 2
		for (text, variable) in self._dbg.items():
			tk.Grid.columnconfigure(self.frame, column, weight=1)
			checkbox = tk.Checkbutton(master=self.frame, text=text, variable=variable)
			if text == "Show FOV":
				checkbox["command"] = self._toggleFov
			if text == "Render Trajectory":
				checkbox["command"] = self._toggleTrajectory
			checkbox.grid(row=1, column=column)
			column += 1

	def condenseGraph(self):
		condensed = self.scenario.fov.cGraphs[self._fovIndex].condense(self.spec)
		GraphAlgorithms.displayGraphAuto(condensed, displayGeomGraph=self.displayGeomGraph, displaySpringGraph=self.displaySpringGraph)

	def validate(self):
		self._validateCallback()
