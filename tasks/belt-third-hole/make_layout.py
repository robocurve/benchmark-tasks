"""Write threaded_layout_<k>.json, the three start layouts (geom.START_LAYOUTS). python3 make_layout.py"""
import json
import pathlib

import numpy as np

import geom as G

# the threaded starts (one per reset seed, geom.START_LAYOUTS): the tail beyond hole 1 is free and rests where
# gravity and its stiffness put it
for idx, (past, hy) in enumerate(G.START_LAYOUTS):
    raw, bow = G._threaded_layout_raw(hinge_y=hy, hole1_past=past)
    first_free = G.HOLE_LINK[1] + 1                     # just past hole 1
    relaxed, e = G.relax_tongue(raw, first_free, hinge_y=hy)
    pathlib.Path(__file__).with_name(f"threaded_layout_{idx}.json").write_text(json.dumps(
        {"joints": np.round(relaxed, 6).tolist(), "bow_m": round(float(bow), 5), "first_free_link": first_free,
         "hole1_past_m": past, "hinge_y_m": hy, "energy_J": round(float(e), 5)}))
    print(f"threaded {idx}: hole 1 {past * 1000:.0f} mm past the hinge, hinge y {hy * 1000:.0f} mm, tip {relaxed[-1]}")

