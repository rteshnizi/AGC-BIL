import tkinter as tk

class Canvas(object):
	def __init__(self, master, app, row, col):
		self.master = master
		self.app = app
		self.tkCanvas = tk.Canvas(master=master)
		self.tkCanvas.grid(row=row, column=col, columnspan=7, sticky=tk.N + tk.S + tk.E + tk.W)

	def render(self, scenario):
		scenario.map.render(self.tkCanvas)
		for storyName in scenario.stories:
			scenario.stories[storyName].render(self.tkCanvas)
