from shapely.geometry import Polygon
import matplotlib.pyplot as plt

p11 = Polygon([[0, 0], [100, 0], [0, 100]])
p12 = Polygon([[100, 0], [100, 100], [0, 100]])
p1 = Polygon([[0, 0], [100, 0], [100, 100], [0, 100]])
p2 = Polygon([[30, 110], [25, 110], [25, -10], [30, -10]])
p3 = Polygon([[70, 110], [75, 110], [-5, 20], [-5, 25]])

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
	result = subPoly.intersection(p2)
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
