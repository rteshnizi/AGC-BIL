from CGAL.CGAL_Kernel import Point_2 as Point
from CGAL.CGAL_Kernel import Polygon_2 as CGPolygon
from CGAL.CGAL_Kernel import intersection
from shapely.geometry import Polygon
import matplotlib.pyplot as plt

# FIXME: This code doesn't work because the intersection API
# Option1: implement API https://doc.cgal.org/latest/Boolean_set_operations_2/group__boolean__intersection.html
# Option2: Hack solution

def getPointArray(verts):
	arr = []
	for vert in verts:
		arr.append(Point(vert[0], vert[1]))
	return arr

def convertToCgal(poly):
	x, y = poly.boundary.xy
	verts = [[x[i], y[i]] for i in range(len(x))]
	cgPoly = CGPolygon(getPointArray(verts))
	return cgPoly

def getXY(poly):
	x = []
	y = []
	for v in poly.vertices():
		x.append(v.x())
		y.append(v.y())
	return x, y

p11 = Polygon([[0, 0], [100, 0], [0, 100]])
p12 = Polygon([[100, 0], [100, 100], [0, 100]])
p1 = Polygon([[0, 0], [100, 0], [100, 100], [0, 100]])
p2 = Polygon([[30, 110], [25, 110], [25, -10], [30, -10]])
p2CG = CGPolygon(getPointArray([[30, 110], [25, 110], [25, -10], [30, -10]]))
p3 = Polygon([[70, 110], [75, 110], [-5, 20], [-5, 25]])
p3CG = CGPolygon(getPointArray([[70, 110], [75, 110], [-5, 20], [-5, 25]]))

fig = plt.figure("Fig-1")
plt.gca().axis("equal")
x,y = p11.exterior.xy
plt.plot(x,y)
x,y = p12.exterior.xy
plt.plot(x,y)
x,y = p2.exterior.xy
plt.plot(x,y)

fig = plt.figure("Fig-2")
plt.gca().axis("equal")
x,y = p11.exterior.xy
plt.plot(x,y)
x,y = p12.exterior.xy
plt.plot(x,y)
x,y = p3.exterior.xy
plt.plot(x,y)

p11AndP2Diff = p11.difference(p2)
print(type(p11AndP2Diff))
p12AndP2Diff = p12.difference(p2)
print(type(p11AndP2Diff))
p11AndP3Diff = p11.difference(p3)
print(type(p11AndP3Diff))
p12AndP3Diff = p12.difference(p3)
print(type(p12AndP3Diff))

fig = plt.figure("(P1 - P2) ⋂ P2")
plt.gca().axis("equal")
for subPoly in list(p11AndP2Diff) + list(p12AndP2Diff):
	x,y = subPoly.boundary.xy
	plt.plot(x,y)
	result = intersection(convertToCgal(subPoly), p2CG)
	print(result.length)
	if not result.is_empty:
		x,y = result.coords.xy
		plt.plot(x,y,"k--", linewidth=3)

fig = plt.figure("(P1 - P3) ⋂ P3")
plt.gca().axis("equal")
for subPoly in list(p11AndP3Diff) + list(p12AndP3Diff):
	x,y = subPoly.boundary.xy
	plt.plot(x,y)
	result = subPoly.intersection(p3)
	print(result.length)
	if not result.is_empty:
		x,y = result.coords.xy
		plt.plot(x,y,"k--", linewidth=3)

plt.show()
