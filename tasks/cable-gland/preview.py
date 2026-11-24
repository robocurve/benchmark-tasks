"""Render the cable-gland starting layout from the station cameras after the parts have settled (no arm motion), and
report the objects' states (does the cable lie still in its curve? score 0 at the start?).

    python preview.py [--out ../../results/cg_preview_N] [--settle 2.0]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
CAMERAS = ("top_cam", "left_cam", "right_cam")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=Path("preview"))
    ap.add_argument("--settle", type=float, default=2.0)
    args = ap.parse_args(argv)

    import cv2
    from inspect_robots import Scene
    from inspect_robots.types import Action

    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    emb = YamArenaEmbodiment(task=str(HERE / "task.py"), control_hz=20.0, object_jitter=0.0, cam_height=360,
                             cam_width=640, device="cpu")
    obs = emb.reset(Scene(id="cable-gland-preview", instruction="preview"), seed=0)
    hold = Action(data=np.asarray(obs.state["joint_pos"], dtype=float))
    from yam_arena.scene import object_states

    task = emb.task.module
    first = object_states(emb._env, emb.task.names)
    res = None
    for _ in range(max(1, round(args.settle * 20))):
        res = emb.step(hold)
    states = object_states(emb._env, emb.task.names)
    args.out.mkdir(parents=True, exist_ok=True)
    for c in CAMERAS:
        cv2.imwrite(str(args.out / f"{c}.png"), np.ascontiguousarray(res.observation.images[c][..., ::-1]))
    import cg_parts as cg

    base = emb._env.unwrapped
    geo = cg.cable_geometry(base, "gland")
    info = {"score": float(task.score(states)),
            "gland": {k: getattr(states["gland"], k) for k in ("turns", "contact_turns", "past_contact",
                                                                "pad_deflection", "crushed", "pull",
                                                                "hands_off_steps")},
            "cable": {k: getattr(states["cable"], k) for k in ("through", "sheath_x", "twist_deg", "tip_x")},
            "cable_moved_mm": round(float(np.linalg.norm(np.asarray(states["cable"].pos)
                                                          - np.asarray(first["cable"].pos))) * 1000, 2),
            "cable_speed": [round(float(v), 4) for v in states["cable"].lin_vel],
            "link_heights_mm": [round(float(p[2]) * 1000, 1) for p in geo["starts"]],
            "hand_dist_mm": round(cg._TRACKS["gland"].hand_dist * 1000, 1),
            "articulations": {k: len(a.body_names) for k, a in base.scene.articulations.items()}}
    (args.out / "preview.json").write_text(json.dumps(info, indent=2, default=float))
    print("PREVIEW " + json.dumps(info, default=float))
    sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
