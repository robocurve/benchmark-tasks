"""The scripted demo for the blink-code module swap task.

Bimanual coordination:
- The left arm operates the DIAG pushbutton on the controller box.
- The right arm manages module manipulation on the bus strip (pulling failed module 3,
  staging it on table, picking the teal spare, and plugging it into socket 3).
- Hands remain separated by >= 200 mm (> 100 mm requirement) at all times.

Waypoint structure: (duration, pos, finger, label, yaw) played open-loop by yam_scripted.
All tuning knobs are exposed as environment variables (DEMO_*):
- DEMO_TIME_SCALE: stretch every waypoint duration (default 1.8)
- DEMO_OVERSHOOT: socket insertion overshoot along +x (m, default 0.0025)
- DEMO_PUSH_S: push insertion duration (s, default 1.4)
- DEMO_INSERT_DZ: raise the insertion height (m, default 0.0006)
- DEMO_PLUG_DY: insertion y offset (m, default 0.0)
- DEMO_STAGE_DY: sideways parking offset of the pulled module (m, default 0.075)
- DEMO_GRIP_DX: right gripper grasp offset along x (m, default 0.0)
- DEMO_GRIP_DY: right gripper grasp offset along y (m, default 0.0)
- DEMO_SETTLE_ONLY: if 1, returns empty plan so arms hold initial ready pose (scores 0.0)
- DEMO_WRONG_SOCKET: if 1, swaps module 2 instead of module 3 (scores 0.0 zero-gate)
- DEMO_NO_DIAG: if 1, swaps module 3 with spare without ever pressing DIAG (scores 0.75)
- DEMO_NO_VERIFY: if 1, initial DIAG press and swap, skips verify press (scores 0.75)
- DEMO_NO_SWAP: if 1, presses DIAG only, no module swap (scores 0.25)
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
else:
    cand = HERE.parent.parent / "refs" / "yam-isaac-lab-arena"
    if cand.exists():
        sys.path.insert(0, str(cand))
sys.path.insert(0, str(HERE))

import geometry as g
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint

PAD_TO_TCP = 0.007
GRIP_PAST_CONTACT = 0.005
HOUSING_TCP_OFFSET_X = -0.009  # centre of housing along world x (origin is nose root)
SHIFT_DEFAULT = {"left": 0.0, "right": 0.0}


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def right_pick_pose(pos, dx=0.0, dy=0.0, dz=0.0):
    grip_dx = _f("DEMO_GRIP_DX", 0.0)
    grip_dy = _f("DEMO_GRIP_DY", 0.0)
    return (pos[0] + HOUSING_TCP_OFFSET_X + dx + grip_dx, pos[1] + dy + grip_dy, pos[2] + PAD_TO_TCP + dz)


def demo_plans() -> dict[str, list[Waypoint]]:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return {}

    mode = "demo"
    if os.environ.get("DEMO_WRONG_SOCKET") == "1":
        mode = "wrong_socket"
    elif os.environ.get("DEMO_NO_DIAG") == "1":
        mode = "no_diag"
    elif os.environ.get("DEMO_NO_VERIFY") == "1":
        mode = "no_verify"
    elif os.environ.get("DEMO_NO_SWAP") == "1":
        mode = "no_swap"

    yaw_r = _f("DEMO_YAW_SHIFT_RIGHT", SHIFT_DEFAULT["right"])
    yaw_l = _f("DEMO_YAW_SHIFT_LEFT", SHIFT_DEFAULT["left"])

    finger_open = FINGER_OPEN
    finger_release = -0.027  # 54 mm opening: clears 45 mm housing by 4.5 mm/side while leaving > 15 mm to adjacent socket
    finger_grip = -g.HOUSING[0] / 2 + GRIP_PAST_CONTACT  # -0.0175
    finger_btn = -0.015      # closed fist for pressing button

    hover_z = g.TOP + 0.12
    press_z = g.BUTTON_PRESS_TCP_Z

    overshoot = _f("DEMO_OVERSHOOT", 0.0025)
    push_s = _f("DEMO_PUSH_S", 1.4)
    # GPU runs: with the old module parked aside, insertion works at +0.6 mm height and no y offset (clearance is 1 mm per
    # side; 0.0 height stalled at the socket mouth, +0.3 mm with -0.5 mm y also worked).
    insert_dz = _f("DEMO_INSERT_DZ", 0.0006)
    plug_dy = _f("DEMO_PLUG_DY", 0.0)

    home_l = (0.180, 0.200, hover_z)
    home_r = (0.180, -0.200, hover_z)
    diag_btn_xy = (g.DIAG_BTN_POS[0], g.DIAG_BTN_POS[1])

    # Target socket for swap: socket 3 (index 2, y = -0.190) in demo; socket 2 (index 1, y = -0.120) in wrong_socket
    target_socket_idx = 1 if mode == "wrong_socket" else 2
    target_socket_y = g.SOCKET_Y[target_socket_idx]
    target_module_tag = f"socket_{target_socket_idx + 1}"

    left: list[Waypoint] = []
    right: list[Waypoint] = []

    def sync(label="sync"):
        dur_l = sum(w.duration for w in left)
        dur_r = sum(w.duration for w in right)
        if dur_l < dur_r:
            p = left[-1].pos if left else home_l
            f = left[-1].finger if left else finger_open
            left.append(Waypoint(dur_r - dur_l, p, f, f"l: wait {label}", yaw_l))
        elif dur_r < dur_l:
            p = right[-1].pos if right else home_r
            f = right[-1].finger if right else finger_open
            right.append(Waypoint(dur_l - dur_r, p, f, f"r: wait {label}", yaw_r))

    # Phase 1: Initial ready poses
    left.append(Waypoint(1.2, home_l, finger_open if mode == "no_diag" else finger_btn, "l: ready", yaw_l))
    right.append(Waypoint(1.2, home_r, finger_open, "r: ready", yaw_r))

    # Phase 2: Concurrent action
    # Left arm operates DIAG button (unless in no_diag mode)
    if mode != "no_diag":
        left.extend([
            Waypoint(1.2, (diag_btn_xy[0], diag_btn_xy[1], hover_z), finger_btn, "l: approach DIAG", yaw_l),
            Waypoint(1.0, (diag_btn_xy[0], diag_btn_xy[1], press_z), finger_btn, "l: press DIAG", yaw_l),
            Waypoint(10.0, (diag_btn_xy[0], diag_btn_xy[1], press_z), finger_btn, "l: hold DIAG blink code", yaw_l),
            Waypoint(1.0, (diag_btn_xy[0], diag_btn_xy[1], hover_z), finger_btn, "l: release DIAG", yaw_l),
            Waypoint(1.2, home_l, finger_btn, "l: return home after blink", yaw_l),
        ])

    # Right arm swaps module (unless in no_swap mode)
    if mode != "no_swap":
        # Unplug target module from its socket
        pick_seated = right_pick_pose((g.MOUTH_X, target_socket_y, g.SOCKET_Z))
        # GPU run 2: the pulled module was parked exactly where the spare starts its push, so the spare came down on top of it
        # and tilted 16 deg. Pull straight back along the socket axis, then park the old module off to the side.
        stage_dy = _f("DEMO_STAGE_DY", 0.075)
        pull_back = right_pick_pose((g.STAGE_X, target_socket_y, g.SOCKET_Z))
        pick_stage = right_pick_pose((g.STAGE_X, target_socket_y + stage_dy, g.STAGE_Z))
        spare_pick = right_pick_pose(g.SPARE_STAGE_POS)
        plug_pre = right_pick_pose((g.STAGE_X, target_socket_y, g.SOCKET_Z), dy=plug_dy, dz=insert_dz)
        plug_target = right_pick_pose((g.MOUTH_X + overshoot, target_socket_y, g.SOCKET_Z), dy=plug_dy, dz=insert_dz)
        plug_seated = right_pick_pose((g.MOUTH_X, target_socket_y, g.SOCKET_Z), dy=plug_dy, dz=insert_dz)
        plug_retract = (g.STAGE_X, target_socket_y, g.SOCKET_Z + PAD_TO_TCP + 0.010)

        right.extend([
            # Approach target socket (use finger_release on descent to clear adjacent seated modules at 70 mm pitch)
            Waypoint(1.2, (pick_seated[0], pick_seated[1], hover_z), finger_open, f"r: hover {target_module_tag}", yaw_r),
            Waypoint(1.0, pick_seated, finger_release, f"r: descend to {target_module_tag}", yaw_r),
            Waypoint(0.8, pick_seated, finger_grip, f"r: grip {target_module_tag}", yaw_r),
            Waypoint(1.0, (pull_back[0], pull_back[1], pick_seated[2]), finger_grip, f"r: pull {target_module_tag}", yaw_r),
            Waypoint(1.0, pick_stage, finger_grip, f"r: lower {target_module_tag} to table", yaw_r),
            Waypoint(0.6, pick_stage, finger_release, f"r: release {target_module_tag}", yaw_r),
            Waypoint(1.0, (pick_stage[0], pick_stage[1], hover_z), finger_release, "r: lift after pull", yaw_r),

            # Pick spare module from staging spot
            Waypoint(1.2, (spare_pick[0], spare_pick[1], hover_z), finger_open, "r: hover spare", yaw_r),
            Waypoint(1.0, spare_pick, finger_open, "r: descend to spare", yaw_r),
            Waypoint(0.8, spare_pick, finger_grip, "r: grip spare", yaw_r),
            Waypoint(1.0, (spare_pick[0], spare_pick[1], hover_z), finger_grip, "r: lift spare", yaw_r),

            # Plug spare into target socket
            Waypoint(1.2, (plug_pre[0], plug_pre[1], hover_z), finger_grip, f"r: hover plug {target_module_tag}", yaw_r),
            Waypoint(1.0, plug_pre, finger_grip, f"r: pre-plug {target_module_tag}", yaw_r),
            Waypoint(push_s, plug_target, finger_grip, f"r: push into {target_module_tag}", yaw_r),
            Waypoint(0.8, plug_target, finger_grip, f"r: hold plug {target_module_tag}", yaw_r),
            Waypoint(0.6, plug_seated, finger_release, f"r: release plug {target_module_tag}", yaw_r),
            Waypoint(1.0, plug_retract, finger_release, "r: retract after plug", yaw_r),
            Waypoint(1.2, home_r, finger_open, "r: return home after swap", yaw_r),
        ])

    # Sync arms after swap & initial blink hold
    sync("after_swap")

    # Phase 3: Verify with DIAG hold (unless no_verify or no_diag or no_swap)
    if mode in ("demo", "wrong_socket"):
        left.extend([
            Waypoint(1.0, (diag_btn_xy[0], diag_btn_xy[1], hover_z), finger_btn, "l: approach DIAG verify", yaw_l),
            Waypoint(1.0, (diag_btn_xy[0], diag_btn_xy[1], press_z), finger_btn, "l: press DIAG verify", yaw_l),
            Waypoint(5.0, (diag_btn_xy[0], diag_btn_xy[1], press_z), finger_btn, "l: hold DIAG verify (green)", yaw_l),
            Waypoint(1.0, (diag_btn_xy[0], diag_btn_xy[1], hover_z), finger_btn, "l: release DIAG verify", yaw_l),
            Waypoint(1.2, home_l, finger_open, "l: return home after verify", yaw_l),
        ])
        sync("after_verify")

    # Phase 4: Final success hold at home poses with both grippers open
    hold_s = 2.5
    left.append(Waypoint(hold_s, home_l, finger_open, "l: final hold", yaw_l))
    right.append(Waypoint(hold_s, home_r, finger_open, "r: final hold", yaw_r))

    # Apply global time scale (1.8x stretch for soft-arm lag)
    scale = _f("DEMO_TIME_SCALE", 1.8)
    if scale != 1.0:
        left[:] = [replace(w, duration=w.duration * scale) for w in left]
        right[:] = [replace(w, duration=w.duration * scale) for w in right]

    return {"left": left, "right": right}


def plans() -> dict[str, list[Waypoint]]:
    return demo_plans()
