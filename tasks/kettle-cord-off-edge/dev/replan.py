"""One-off edit of kettle_cord.py and layout.py: no learning tower, kettle back from the edges, the demo pinches the
kettle-side strand (kept for the record)."""
import pathlib, re
root = pathlib.Path(__file__).resolve().parent.parent
p = root / "kettle_cord.py"
s = p.read_text()
start = s.index("def _tower_shapes() -> list:")
end = s.index("CORD_NODES, LOOP_DEPTH = L.cord_nodes()")
s = s[:start] + s[end:]
rep = [
    ("A 1.7 L electric kettle (filled, lid shut) stands on its cordless power base near the counter's front edge (the table's\n"
     "left edge). The base's cord runs to the edge, hangs over it in a U, comes back up over the edge and runs across the\n"
     "counter to its plug in a socket strip. A toddler's sippy cup stands on the counter and a toddler's learning tower is\n"
     "pushed against the front edge.",
     "A 1.7 L electric kettle (filled, lid shut) stands on its cordless power base well back from the counter's edges (the\n"
     "table's left edge is the front edge). The base's cord runs to the front edge, hangs over it in a U, comes back up over\n"
     "the edge and runs across the counter to its plug in a socket strip. A toddler's sippy cup stands on the counter."),
    ("TOWER_C, STEP_C = (0.96, 0.96, 0.94), (0.55, 0.80, 0.45)\n", ""),
    ("TOWER_POS = np.array([(L.TOWER_X[0] + L.TOWER_X[1]) / 2, L.EDGE_Y + 0.005, 0.0])\n", ""),
    ('    tower = Body.build("tower", pos=tuple(TOWER_POS), mass=8.0, shapes=_tower_shapes(), color=TOWER_C, kinematic=True)\n', ""),
    ("    out = [base, jug, lever, strip, cup, tower]", "    out = [base, jug, lever, strip, cup]"),
    (',\n           "tower": (TOWER_POS, ident)}', "}"),
    ('PINCH_LINK = int(os.environ.get("KETTLE_PINCH", "-1"))  # -1: the plug-side link nearest the edge on the counter',
     'PINCH_LINK = int(os.environ.get("KETTLE_PINCH", "-1"))  # -1: the base-side link nearest the edge on the counter'),
    ('PULL_TO = (float(os.environ.get("KETTLE_PULL_X", "0.13")), float(os.environ.get("KETTLE_PULL_Y", "0.38")))',
     'PULL_TO = (float(os.environ.get("KETTLE_PULL_X", "0.25")), float(os.environ.get("KETTLE_PULL_Y", "0.47")))'),
    ("        if i > len(LINKS) // 2 and p[1] < L.EDGE_Y - 0.045 and p[2] > TOP - 0.002 and (best is None or p[1] > best):",
     "        if i < len(LINKS) // 2 and p[1] < L.EDGE_Y - 0.07 and p[2] > TOP - 0.002 and (best is None or p[1] > best):"),
    ('                 _wp(2.0, g + up * 0.10, CORD_SQUEEZE, "lift the cord", r),\n'
     '                 _wp(2.5, np.array([g[0] - 0.01, (g[1] + tgt[1]) / 2 + 0.04, TOP + LIFT]), CORD_SQUEEZE,\n'
     '                     "draw the cord up", r),',
     '                 _wp(2.0, g + np.array([-0.03, -0.02, 0.10]), CORD_SQUEEZE, "lift the cord", r),\n'
     '                 _wp(2.5, np.array([(g[0] + tgt[0]) / 2, (g[1] + tgt[1]) / 2 + 0.02, TOP + LIFT]), CORD_SQUEEZE,\n'
     '                     "draw the cord up", r),'),
    ('        high = np.array([g[0], g[1] - 0.06, TOP + 0.12])  # behind the lever, on the side away from the handle\n'
     '        left += [_wp(3.0, high, 0.0, "above the switch", rp),',
     '        high = np.array([g[0], g[1], TOP + 0.12])  # over the lever tip, beside the handle\n'
     '        around = np.array([0.22, 0.20, TOP + 0.12])  # round the kettle on the robot side\n'
     '        left += [_wp(2.5, around, 0.0, "round the kettle", rp),\n'
     '                 _wp(2.0, high, 0.0, "above the switch", rp),'),
]
for a, b in rep:
    assert a in s, a[:80]
    s = s.replace(a, b)
assert "tower" not in s.lower(), [m.start() for m in re.finditer("tower", s.lower())]
p.write_text(s)
q = root / "layout.py"
t = q.read_text()
for a, b in [("BASE_XY = (0.40, 0.31)", "BASE_XY = (0.38, 0.31)"),
             ("CORD_LEN = 0.76  # 76 cm (estimate: the short cord the safeguards ask for)",
              "CORD_LEN = 0.70  # 70 cm (estimate; the manual gives no length)")]:
    assert a in t, a
    t = t.replace(a, b)
q.write_text(t)
print("ok")
