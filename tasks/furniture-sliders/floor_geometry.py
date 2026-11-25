"""Watertight notched panel with shared vertices and no internal vertical faces.

One static triangle mesh represents the continuous physical panel. Separate
touching box colliders created a false vertical snag at their internal seam.
This module has no simulator dependencies.
"""


def panel_mesh(center, size):
    cx, cy, _ = center
    sx, sy, sz = size
    xs = (cx-sx/2, .10, cx+sx/2)
    ys = (cy-sy/2, -.415, -.195, .195, .415, cy+sy/2)
    cells = {(i, j) for i in range(2) for j in range(5)
             if i == 1 or j in (0, 2, 4)}
    vertices, indices, faces, edges = [], {}, [], {}

    def vertex(i, j, top):
        key = (i, j, top)
        if key not in indices:
            indices[key] = len(vertices)
            vertices.append((xs[i]-cx, ys[j]-cy, (1 if top else -1)*sz/2))
        return indices[key]

    for i, j in sorted(cells):
        corners = ((i, j), (i+1, j), (i+1, j+1), (i, j+1))
        top = [vertex(*p, True) for p in corners]
        bottom = [vertex(*p, False) for p in corners]
        faces += [(top[0], top[1], top[2]), (top[0], top[2], top[3]),
                  (bottom[2], bottom[1], bottom[0]), (bottom[3], bottom[2], bottom[0])]
        for a, b in zip(corners, corners[1:]+corners[:1]):
            key = tuple(sorted((a, b)))
            if key in edges:
                del edges[key]
            else:
                edges[key] = (a, b)
    for a, b in edges.values():
        lo_a, lo_b = vertex(*a, False), vertex(*b, False)
        hi_a, hi_b = vertex(*a, True), vertex(*b, True)
        faces += [(lo_a, lo_b, hi_b), (lo_a, hi_b, hi_a)]
    return vertices, faces
