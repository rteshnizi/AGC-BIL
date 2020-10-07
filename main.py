import sys

from bil.bil import Bil
from ttl.ttl import Ttl

def main(module=None):
	if module is None:
		bil = Bil()
		ttl = Ttl(bil=bil)
		ttl.run()
	elif module == "bil":
		bil = Bil(loadFromFile=True)
		bil.run()
	elif module == "ttl":
		ttl = Ttl(bil=None)
		ttl.run()
	else:
		raise RuntimeError("Unknown Module name: %s. Expected bil or ttl" % module)

if __name__ == '__main__':
	module = None
	if len(sys.argv) > 1:
		module = sys.argv[1]
	main(module=module)
