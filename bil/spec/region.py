from bil.model.pose import Pose
from bil.spec.symbol import SpecAction

class RegionValidation:
	@staticmethod
	def query(p: Pose, symbol: SpecAction) -> bool:
		raise RuntimeError("Not implemented")
