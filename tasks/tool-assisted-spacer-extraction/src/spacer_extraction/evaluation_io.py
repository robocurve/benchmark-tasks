"""Pure evaluation helpers; importing them does not import Isaac or Inspect."""
from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np
from yam_arena import ik

RUN_LIMIT_STEPS = 2400


def evaluator_fingerprint() -> str:
    digest = hashlib.sha256()
    for name in ("task.py", "tracker.py", "runtime.py", "evaluation_io.py"):
        digest.update(name.encode())
        digest.update(Path(__file__).with_name(name).read_bytes())
    return digest.hexdigest()


def release_state(objects: dict, joint_pos, bases: dict) -> dict:
    """Require open jaws and TCPs 80 mm from both complete object bounds."""
    joints = np.asarray(joint_pos, dtype=float)
    if joints.shape != (14,) or not np.all(np.isfinite(joints)):
        raise ValueError("release check requires 14 finite measured joint values")
    clearances = []
    tcp_positions = []
    for arm, offset in (("left", 0), ("right", 7)):
        base = np.asarray(bases[arm], dtype=float)
        if base.shape != (3,) or not np.all(np.isfinite(base)):
            raise ValueError("invalid robot base position")
        tcp, _ = ik.tcp_pose(joints[offset : offset + 6], base)
        tcp = np.asarray(tcp, dtype=float)
        tcp_positions.append(tcp.tolist())
        distances = []
        for name in ("pusher", "spacer"):
            state = objects[name]
            low, high = np.asarray(state.low), np.asarray(state.high)
            if low.shape != (3,) or high.shape != (3,) or not np.all(np.isfinite([low, high])) or np.any(low > high):
                raise ValueError("invalid object bounds for release check")
            distances.append(float(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0))))
        clearances.append(min(distances))
    openings = [float(joints[6]), float(joints[13])]
    released = min(openings) >= 0.8 and min(clearances) >= 0.08
    return {"released": bool(released), "openings": openings,
            "tcp_positions": tcp_positions, "clearances_m": clearances}
