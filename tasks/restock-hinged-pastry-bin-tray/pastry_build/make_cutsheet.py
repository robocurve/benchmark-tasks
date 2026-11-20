"""Writes cutsheet.svg: the flat parts of the real-world bakery bin (mm), laid out for laser cutting or saw cutting.
Dimensions are the ones in the simulation (task.py). Run: python make_cutsheet.py"""
parts = [  # name, qty, w, h (mm), thickness (mm), material
    ("Floor slab", 1, 200, 200, 8, "stainless steel plate"),
    ("Rear wall", 1, 200, 60, 8, "cast acrylic, clear"),
    ("Side wall", 2, 200, 54, 8, "cast acrylic, clear"),
    ("Front wall", 1, 184, 42, 8, "cast acrylic, clear"),
    ("Lid top", 1, 194, 196, 7, "cast acrylic, clear"),
    ("Lid front skirt", 1, 196, 14, 7, "cast acrylic, clear"),
    ("Lid side skirt", 2, 194, 12, 7, "cast acrylic, clear"),
    ("Lid stiffener", 1, 186, 18, 6, "stainless steel strip"),
    ("Handlebar", 1, 228, 12, 12, "stainless steel bar"),
    ("Handlebar standoff", 2, 18, 6, 8, "stainless steel"),
    ("Hinge block", 2, 14, 14, 16, "stainless steel block, bored for hinge pin"),
]
x = y = 10; rowh = 0; out = []; W = 520
for name, q, w, h, t, mat in parts:
    for i in range(q):
        if x + w > W: x = 10; y += rowh + 24; rowh = 0
        out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="none" stroke="#111" stroke-width="0.6"/>')
        out.append(f'<text x="{x+2}" y="{y+h/2+2}" font-size="5" font-family="sans-serif">{name} {w}x{h}x{t} ({mat})</text>')
        x += w + 10; rowh = max(rowh, h)
H = y + rowh + 20
open("cutsheet.svg", "w").write(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H}mm" viewBox="0 0 {W} {H}">\n' + "\n".join(out) + "\n</svg>\n")
print("wrote cutsheet.svg with", sum(p[1] for p in parts), "parts")
