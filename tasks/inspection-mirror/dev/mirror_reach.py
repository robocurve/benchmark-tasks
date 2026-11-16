"""Offline reach check of every scripted waypoint on the stock yam_arena IK (gate: <0.5 mm, <0.1 deg). No GPU."""
import numpy as np
from yam_arena import ik, station
from yam_arena.constants import READY

from inspection_mirror.policy import InspectionMirrorScriptedPolicy

pol = InspectionMirrorScriptedPolicy()
pol._bases = dict(station.BASES)
bad = 0
for arm, label, pos, rot in pol.waypoints():
    q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(READY, float),
                         base_pos=np.asarray(station.BASES[arm], float), iters=400)
    ok = pe * 1000 < 0.5 and np.degrees(re) < 0.1
    bad += not ok
    print(f"{'OK ' if ok else 'BAD'} {arm:5s} {label:32s} pos={np.round(pos, 3).tolist()} pe={pe*1000:7.2f} mm "
          f"re={np.degrees(re):6.2f} deg")
print("unreachable:", bad)
