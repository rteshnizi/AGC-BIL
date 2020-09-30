class Ttl(object):
	def __init__(self, bil=None):
		self.bil = bil

	def run(self):
		for i in range(10):
			print("I am updating my data. tick %d" % i)
			if self.bil is not None:
				self.bil.update("Here is new trajectory...")
