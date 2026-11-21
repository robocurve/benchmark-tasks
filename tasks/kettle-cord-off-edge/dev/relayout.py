"""One-off edit of layout.py for the visible-crossing layout (kept for the record)."""
import pathlib
p = pathlib.Path(__file__).resolve().parent.parent / "layout.py"
s = p.read_text()
rep = [
    ('BASE_XY = (0.50, 0.52)  # the power base\'s centre: its rim is 5 cm from the front edge',
     'BASE_XY = (0.40, 0.31)  # the power base\'s centre: its rim 26 cm from the front edge (ABA: hot items "at least 10\n#                         inches from the edge" of a table)'),
    ('EXIT = (BASE_XY[0] - 0.072, BASE_XY[1] + 0.030)  # the cord leaves the base on its side toward the robot',
     'EXIT = (BASE_XY[0], BASE_XY[1] + 0.078)  # the cord leaves the base on its side toward the front edge'),
    ('A_X, B_X = 0.40, 0.32  #', 'A_X, B_X = 0.45, 0.58  #'),
    ('STRIP_XY = (0.30, 0.27)  #', 'STRIP_XY = (0.72, 0.53)  #'),
    ('CUP_XY, CUP_R, CUP_H, CUP_MASS = (0.52, 0.20),', 'CUP_XY, CUP_R, CUP_H, CUP_MASS = (0.62, 0.15),'),
    ("TOWER_X = (0.47, 0.89)  # the learning tower's side along the front edge; it stands outside the counter (y > 0.65)\n"
     "TOWER_DEPTH, TOWER_RAIL_Z = 0.55, 0.90  # its top rail 90 cm above the floor: 15 cm over the counter\n", ""),
    ("    rad = (A_X - B_X) / 2\n    zb = TOP - depth + rad\n", "    rad = (A_X - B_X) / 2\n    zb = TOP - depth + abs(rad)\n"),
    ("zb - rad * math.sin(a)", "zb - abs(rad) * math.sin(a)"),
    ("    pts += [np.array([PLUG_XY[0] - 0.02, PLUG_XY[1] + 0.07, z0]), np.array([PLUG_XY[0], PLUG_XY[1] + 0.02, TOP + PLUG_Z])]",
     "    pts += [np.array([B_X + 0.01, PLUG_XY[1] + 0.03, z0]), np.array([PLUG_XY[0] - 0.035, PLUG_XY[1], z0 + 0.01]),\n"
     "            np.array([PLUG_XY[0], PLUG_XY[1], TOP + PLUG_Z])]"),
    ("Sippy cup: Munchkin Miracle 360 trainer cup, 7 oz. Learning tower: a toddler kitchen helper tower (standing platform\n"
     "  with rails), pushed against the counter's front edge.", "Sippy cup: Munchkin Miracle 360 trainer cup, 7 oz."),
    ("the cup's and the tower's sizes", "the cup's size"),
]
for a, b in rep:
    assert a in s, a[:70]
    s = s.replace(a, b)
p.write_text(s)
print("layout.py rewritten")
