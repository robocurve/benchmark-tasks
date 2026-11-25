"""Convex, rounded prescription and sun lenses. The optical axis is local X."""
import math


def vertices(size, radius):
    thickness, width, height = size
    outline = []
    for sy, sz, quadrant in [(1, 1, 0), (-1, 1, 1), (-1, -1, 2), (1, -1, 3)]:
        y = sy * (width / 2 - radius)
        z = sz * (height / 2 - radius)
        for step in range(5):
            theta = math.pi * (quadrant / 2 + step / 8)
            outline.append((y + radius * math.cos(theta), z + radius * math.sin(theta)))
    return [(x, y, z) for x in (-thickness / 2, thickness / 2) for y, z in outline]


def geometry(size, radius):
    from .scene import triple
    points = vertices(size, radius)
    n = len(points) // 2
    faces = [list(reversed(range(n))), list(range(n, 2 * n))]
    for i in range(n):
        nxt = (i + 1) % n
        faces.append([i, nxt, nxt + n, i + n])
    counts = ", ".join(str(len(face)) for face in faces)
    indices = ", ".join(str(index) for face in faces for index in face)
    points_text = ", ".join(triple(point) for point in points)
    return f'''point3f[] points = [{points_text}]
            int[] faceVertexCounts = [{counts}]
            int[] faceVertexIndices = [{indices}]
            uniform token subdivisionScheme = "none"
            uniform token physics:approximation = "convexHull"'''
