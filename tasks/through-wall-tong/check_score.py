"""The score lines on hand-built states, with no simulator: python3 check_score.py. Runs `judge` from task.py, read from its source."""

import ast
import pathlib
import sys

src = pathlib.Path(__file__).with_name("task.py").read_text()
ns = {"HOLDER_R": None, "CLEAR": None}
for node in ast.parse(src).body:
    if isinstance(node, ast.Assign) and any(n.id in ("HOLDER_R", "CLEAR") for t in node.targets for n in ast.walk(t) if isinstance(n, ast.Name)):
        exec(ast.get_source_segment(src, node).split("  #")[0], ns)
    if isinstance(node, ast.FunctionDef) and node.name == "judge":
        exec("from __future__ import annotations\n" + ast.get_source_segment(src, node), ns)
judge = ns["judge"]
RACK_OFF = 0.18  # the rack stands this far from the holder's axis

# name: (latched failure, off axis, base height, tilt, still, gaps of the two jaws), wanted (full, score)
CASES = {
    "standing in the holder, jaws 3.2 cm away, still": ((None, 0.002, 0.000, 0.0, True, (0.032, 0.033)), (True, 0.5)),
    "standing in the holder, jaws just under 3 cm": ((None, 0.002, 0.000, 0.0, True, (0.029, 0.033)), (False, 0.5)),
    "standing in the holder, both jaws on it": ((None, 0.002, 0.003, 0.5, True, (0.0, 0.0)), (False, 0.5)),
    "in the holder, jaws clear, but still rocking": ((None, 0.002, 0.000, 0.5, False, (0.04, 0.04)), (False, 0.5)),
    "leaning in the holder, past 5 degrees": ((None, 0.004, 0.002, 6.0, True, (0.05, 0.05)), (False, 0.0)),
    "held up by both jaws": ((None, 0.18, 0.054, 10.0, False, (0.0, 0.001)), (False, 0.25)),
    "on the rim, one pad beside it, jaws open": ((None, 0.02, 0.040, 3.0, True, (0.002, 0.024)), (False, 0.0)),
    "on the bench, not in the holder": ((None, 0.09, 0.000, 0.0, True, (0.05, 0.05)), (False, 0.0)),
    "back in the rack": ((None, RACK_OFF, 0.000, 0.0, True, (0.05, 0.05)), (False, 0.0)),
    "pushed along the floor to the holder's wall": (("tipped", 0.05, 0.010, 60.0, True, (0.05, 0.05)), (False, 0.0)),
    "fell into the holder: seated, but latched": (("fell", 0.002, 0.000, 0.0, True, (0.05, 0.05)), (False, 0.0)),
    "knocked over earlier, then held up": (("tipped", 0.1, 0.054, 10.0, False, (0.0, 0.0)), (False, 0.0)),
}
bad = 0
for name, (args, want) in CASES.items():
    full, score, *_ = judge(*args)
    good = (full, score) == want
    bad += not good
    print(f"{'ok ' if good else 'BAD'} {name}: full {full}, score {score}")
print("all good" if not bad else f"{bad} FAILED")
sys.exit(1 if bad else 0)
