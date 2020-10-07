from

class Bil(object):
	def __init__(self, loadFromFile=False):
		self.loadFromFile = loadFromFile
		self.parser = Parser

	def run(self):
		pass

	def update(self, data):
		print("BIL says: %s" % repr(data))
