#!/usr/bin/env python3
"""Offline URDF reachability only; this neither simulates contacts nor proves success."""

import json

from camera_release.controller import ScriptedPolicy

if __name__ == "__main__":
    rows = []
    for mode in ("hold", "open_only", "direct", "demo", "drop"):
        policy = ScriptedPolicy(mode=mode)
        policy.reset(None)
        for step in range(round(policy.duration*20)+1):
            policy.action_at(step/20)
        rows.append(dict(mode=mode, duration_s=policy.duration,
                         worst_ik_position_m=policy.worst_position_error,
                         worst_ik_rotation_rad=policy.worst_rotation_error))
    print(json.dumps(dict(scope="offline kinematics only", plans=rows), indent=2))
