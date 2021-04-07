from typing import Union

from CGAL.CGAL_Kernel import cross_product as _cross_product
from CGAL.CGAL_Kernel import intersection as _intersection
from CGAL.CGAL_Kernel import Line_2 as Line
from CGAL.CGAL_Kernel import Point_2 as Point
from CGAL.CGAL_Kernel import Polygon_2 as Polygon
from CGAL.CGAL_Kernel import Ref_int as IntRef
from CGAL.CGAL_Kernel import Ray_2 as Ray
from CGAL.CGAL_Kernel import Segment_2 as Segment
from CGAL.CGAL_Kernel import Vector_2 as Vector
from CGAL.CGAL_Kernel import Vector_3 as Vector3D

from CGAL.CGAL_Triangulation_2 import Constrained_Delaunay_triangulation_2 as CgalTriangulation
from CGAL.CGAL_Triangulation_2 import Ref_Constrained_Delaunay_triangulation_2_Face_handle as TriangulationFaceRef
from CGAL.CGAL_Triangulation_2 import Constrained_Delaunay_triangulation_2_Face_handle as TriangulationFaceHandle

from CGAL.CGAL_Convex_hull_2 import ch_graham_andrew as ConvexHull

PointOrSegmentNone = Union[Point, Segment, None]

def pointStringId(x: float, y: float) -> str:
	return "%.5f,%.5f" % (x, y)

def crossProduct(vec1: Vector, vec2: Vector) -> Vector3D:
	vec1_3d = Vector3D(vec1.x(), vec1.y(), 0)
	vec2_3d = Vector3D(vec2.x(), vec2.y(), 0)
	return _cross_product(vec1_3d, vec2_3d)

def intersection(obj1, obj2) -> PointOrSegmentNone:
	obj = _intersection(obj1, obj2)
	if obj.is_Segment_2():
		seg: Segment = obj.get_Segment_2()
		return seg
	if obj.is_Point_2():
		pt: Point = obj.get_Point_2()
		return pt
	return None

# region _repr fixers

# More human readable __repr__ for Point_2
def __Pt2Repr(self):
	return "P(%d, %d)" % (self.x(), self.y())
Point.__repr__ = __Pt2Repr
# We don't need the has anymore, I just kept it for reference
# Make Point_2 hashable
def __Pt2Hash(self):
	return hash(pointStringId(self))
Point.__hash__ = __Pt2Hash

# More human readable __repr__ for Ray_2
def __Ray2Repr(self: Ray):
	return "Ray(%s)" % (repr(self.source()))
Ray.__repr__ = __Ray2Repr


# More human readable __repr__ for Ray_2
def __Seg2Repr(self: Segment):
	return "Seg(%s, %s)" % (repr(self.source()), repr(self.target()))
Segment.__repr__ = __Seg2Repr

# More human readable __repr__ for Vector_2
def __Vec2Repr(self):
	return "Vec(%d, %d)" % (self.x(), self.y())
Vector.__repr__ = __Vec2Repr

# More human readable __repr__ for Vector_3
def __Vec3Repr(self):
	return "Vec3(%d, %d, %d)" % (self.x(), self.y(), self.z())
Vector3D.__repr__ = __Vec3Repr

# endregion
