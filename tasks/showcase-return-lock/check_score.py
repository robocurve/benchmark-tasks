"""The scorer's tier logic and its gripper-clearance test on hand-built states, with no simulator: python3 check_score.py.
Reads moment, box_gap and the thresholds from task.py's source."""

import ast
import math
import pathlib

import numpy as np

src = pathlib.Path(__file__).with_name("task.py").read_text()
ns = {"np": np, "math": math, "TOP": 0.0}
for node in ast.parse(src).body:
    if isinstance(node, ast.FunctionDef) and node.name in ("moment", "box_gap"):
        exec(ast.get_source_segment(src, node), ns)
    if isinstance(node, ast.Assign) and all(n.id.isupper() for t in node.targets for n in ast.walk(t) if isinstance(n, ast.Name)):
        try:
            exec(ast.get_source_segment(src, node), ns)
        except Exception:
            pass
moment, box_gap = ns["moment"], ns["box_gap"]
base = {"failed": False, "on_spot": True, "inner_shut": True, "outer_shut": True, "latched": True, "hands_off": True}
cases = [
    ("everything done, hands off", {}, 1.0),
    ("nothing done (box outside)", {"on_spot": False, "inner_shut": False, "outer_shut": False, "latched": False}, 0.0),
    ("doors shut and locked, box left outside", {"on_spot": False}, 0.0),
    ("box on spot only", {"inner_shut": False, "outer_shut": False, "latched": False}, 0.25),
    ("box on spot, outer shut, inner open (outer first)", {"inner_shut": False, "latched": False}, 0.25),
    ("box and inner door", {"outer_shut": False, "latched": False}, 0.5),
    ("both doors, no lock", {"latched": False}, 0.75),
    ("locked, a hand still on the case", {"hands_off": False}, 0.75),
    ("box dropped or tipped, rest done", {"failed": True}, 0.0),
    ("a watch stand on the higher step knocked over or moved over 1.5 cm, rest done", {"failed": True}, 0.0),
    ("box left on the higher step (not on its spot), rest done", {"on_spot": False}, 0.0),
]
bad = 0
for name, change, want in cases:
    got = moment({**base, **change})
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {got:.2f} (want {want:.2f})  {name}")
lo, hi = ns["CASE_LO"], ns["CASE_HI"]
for name, p, want in [("fingertip touching the lock head's face", (lo[0] - 0.002, 0.025, 0.12), False),
                      ("palm 2 cm back from the pull", (lo[0] - 0.02, 0.06, 0.24), True),
                      ("wrist over the case top", (0.45, 0.0, hi[2] + 0.005), False)]:
    got = box_gap(np.array([p]), lo, hi) >= ns["HANDS_OFF"]
    bad += got != want
    print(f"{'ok ' if got == want else 'BAD'} hands_off {got} (want {want})  {name}")
print("all ok" if not bad else f"{bad} BAD")
