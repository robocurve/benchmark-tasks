"""Scripted bimanual demo and ablations for buffer socket module permutation.

Bimanual coordination:
- The right arm manages module manipulation on the bus strip (pulling modules,
  carrying them to target sockets or buffer, and inserting them).
- The left arm operates the COMMIT pushbutton on the controller box once
  the permutation is complete and the STATUS lamp is green.
- Hands remain separated by >= 200 mm (> 100 mm requirement) at all times.

Waypoint structure: (duration, pos, finger, label, yaw) played open-loop by yam_scripted.
All tuning knobs are exposed as environment variables (DEMO_*):
- DEMO_TIME_SCALE: stretch every waypoint duration (default 1.8)
- DEMO_OVERSHOOT: socket insertion overshoot along +x (m, default 0.005, GPU run 2)
- DEMO_PUSH_S: push insertion duration (s, default 0.9)
- DEMO_INSERT_DZ: raise insertion height (m, default 0.002, GPU run 2)
- DEMO_PLUG_DY: insertion y offset (m, default 0.0)
- DEMO_GRIP_DX: right gripper grasp offset along x (m, default 0.0)
- DEMO_GRIP_DY: right gripper grasp offset along y (m, default 0.0)
- DEMO_YAW_SHIFT_LEFT: yaw shift for left arm (rad, default 0.0)
- DEMO_YAW_SHIFT_RIGHT: yaw shift for right arm (rad, default 0.0)
- DEMO_SETTLE_ONLY: if 1, returns 8 s hold at home (scores 0.0)
- DEMO_NO_COMMIT: if 1, completes 3-move permutation but skips COMMIT press (scores 0.75)
- DEMO_NO_BUFFER: if 1, attempts direct swap without buffer, leaving Green on table (scores 0.50)
"""

from __future__ import annotations

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


def _f(name: str, default: float) -> float:
    try:
        return float(os.environ[name])
    except (KeyError, ValueError):
        return default


def _b(name: str, default: bool = False) -> bool:
    return os.environ.get(name, "1" if default else "0") in ("1", "true", "True")


def mode_from_env() -> str:
    if _b("DEMO_SETTLE_ONLY"):
        return "settle"
    if _b("DEMO_NO_BUFFER"):
        return "no_buffer"
    if _b("DEMO_NO_COMMIT"):
        return "no_commit"
    return "demo"


def right_pick_pose(pos, dx=0.0, dy=0.0, dz=0.0):
    grip_dx = _f("DEMO_GRIP_DX", 0.0)
    grip_dy = _f("DEMO_GRIP_DY", 0.0)
    return (
        pos[0] + HOUSING_TCP_OFFSET_X + dx + grip_dx,
        pos[1] + dy + grip_dy,
        pos[2] + PAD_TO_TCP + dz,
    )


def demo_plans(mode: str | None = None, with_meta: bool = False) -> dict[str, list[Waypoint]] | tuple[dict[str, list[Waypoint]], dict]:
    mode = mode_from_env() if mode is None else mode

    yaw_l = _f("DEMO_YAW_SHIFT_LEFT", 0.0)
    yaw_r = _f("DEMO_YAW_SHIFT_RIGHT", 0.0)

    finger_open = FINGER_OPEN
    finger_release = -0.027  # 54 mm opening: clears 45 mm housing by 4.5 mm/side while leaving > 15 mm to adjacent socket
    finger_grip = -g.HOUSING[0] / 2 + GRIP_PAST_CONTACT  # -0.0175
    finger_btn = -0.015      # closed fist for pressing COMMIT button

    hover_z = g.TOP + 0.12
    press_z = g.BUTTON_PRESS_TCP_Z

    overshoot = _f("DEMO_OVERSHOOT", 0.005)
    push_s = _f("DEMO_PUSH_S", 0.9)
    insert_dz = _f("DEMO_INSERT_DZ", 0.002)
    plug_dy = _f("DEMO_PLUG_DY", 0.0)

    home_l = (0.180, 0.200, hover_z)
    home_r = (0.180, -0.200, hover_z)

    left: list[Waypoint] = []
    right: list[Waypoint] = []
    meta = {}

    def add(arm: str, duration: float, pos, finger: float, label: str, op: int, kind: str, **kwargs):
        yaw = yaw_l if arm == "left" else yaw_r
        wp = Waypoint(duration, tuple(float(v) for v in pos), finger, label, yaw)
        if arm == "left":
            left.append(wp)
        else:
            right.append(wp)
        meta[label] = dict(arm=arm, op=op, kind=kind, **kwargs)

    # Mode: settle -> explicit 8.0 s hold at home with open grippers
    if mode == "settle":
        hold_s = _f("DEMO_SETTLE_HOLD_S", 8.0)
        add("left", hold_s, home_l, finger_open, "l: settle hold", 0, "hold")
        add("right", hold_s, home_r, finger_open, "r: settle hold", 0, "hold")
        out = {"left": left, "right": right}
        return (out, meta) if with_meta else out

    def sync(label: str, op: int):
        dur_l = sum(w.duration for w in left)
        dur_r = sum(w.duration for w in right)
        if dur_l < dur_r - 1e-6:
            p = left[-1].pos if left else home_l
            f = left[-1].finger if left else finger_open
            add("left", dur_r - dur_l, p, f, f"l: wait {label}", op, "wait")
        elif dur_r < dur_l - 1e-6:
            p = right[-1].pos if right else home_r
            f = right[-1].finger if right else finger_open
            add("right", dur_l - dur_r, p, f, f"r: wait {label}", op, "wait")

    op = 0

    # Phase 1: Initial ready poses
    add("left", 1.0, home_l, finger_btn, "l: ready", op, "transit")
    add("right", 1.0, home_r, finger_open, "r: ready", op, "transit")
    sync("ready", op)
    op += 1

    # Helpers for module socket pick & plug
    # Sockets:
    # 0: Socket 1 (y = -0.260)
    # 1: Socket 2 (y = -0.190)
    # 2: Socket 3 (y = -0.120)
    # 3: Socket 4 (y = -0.050, buffer)

    def module_transfer(from_sock: int, to_sock: int, mod_name: str, op_idx: int):
        from_y = g.SOCKET_Y[from_sock]
        to_y = g.SOCKET_Y[to_sock]

        pick_seated = right_pick_pose((g.MOUTH_X, from_y, g.SOCKET_Z))
        pull_back = right_pick_pose((g.STAGE_X, from_y, g.SOCKET_Z))

        plug_pre = right_pick_pose((g.STAGE_X, to_y, g.SOCKET_Z), dy=plug_dy, dz=insert_dz)
        plug_target = right_pick_pose((g.MOUTH_X + overshoot, to_y, g.SOCKET_Z), dy=plug_dy, dz=insert_dz)
        plug_seated = right_pick_pose((g.MOUTH_X, to_y, g.SOCKET_Z), dy=plug_dy, dz=insert_dz)
        plug_retract = (g.STAGE_X, to_y, g.SOCKET_Z + PAD_TO_TCP + 0.010)

        # Pull from from_sock
        add("right", 0.7, (pick_seated[0], pick_seated[1], hover_z), finger_release, f"r: hover {mod_name} s{from_sock+1}", op_idx, "hover")
        add("right", 0.7, pick_seated, finger_release, f"r: descend to {mod_name} s{from_sock+1}", op_idx, "descend")
        add("right", 0.5, pick_seated, finger_grip, f"r: grip {mod_name} s{from_sock+1}", op_idx, "grip")
        add("right", 0.7, (pull_back[0], pull_back[1], pick_seated[2]), finger_grip, f"r: pull {mod_name} s{from_sock+1}", op_idx, "pull")
        add("right", 0.6, (pull_back[0], pull_back[1], hover_z), finger_grip, f"r: lift {mod_name} after pull", op_idx, "lift")

        # Transit to to_sock
        add("right", 0.8, (plug_pre[0], plug_pre[1], hover_z), finger_grip, f"r: transit {mod_name} to s{to_sock+1}", op_idx, "transit")
        add("right", 0.7, plug_pre, finger_grip, f"r: descend plug approach s{to_sock+1}", op_idx, "descend")
        add("right", push_s, plug_target, finger_grip, f"r: push {mod_name} into s{to_sock+1}", op_idx, "push", from_sock=from_sock, to_sock=to_sock, mod=mod_name)
        add("right", 0.4, plug_target, finger_grip, f"r: dwell plug s{to_sock+1}", op_idx, "hold")
        add("right", 0.4, plug_seated, finger_release, f"r: release plug s{to_sock+1}", op_idx, "release")
        add("right", 0.6, plug_retract, finger_release, f"r: retract from s{to_sock+1}", op_idx, "retract")
        add("right", 0.6, (plug_retract[0], plug_retract[1], hover_z), finger_release, f"r: lift after plug s{to_sock+1}", op_idx, "lift")

    if mode == "no_buffer":
        # Ablation: attempts direct swap without buffer.
        # Pulls Green from Socket 1, parks it on the table on the robot side (y = -0.340),
        # then moves Red from Socket 2 into Socket 1. Green remains on table unseated -> 0.50 score.
        pick_s1 = right_pick_pose((g.MOUTH_X, g.SOCKET_Y[0], g.SOCKET_Z))
        pull_s1 = right_pick_pose((g.STAGE_X, g.SOCKET_Y[0], g.SOCKET_Z))
        # GPU run: x=0.380 is past the sockets (the hand pushed the whole fixture 30 mm and tripped the fixture-moved gate).
        # Park the pulled module on the robot side of the fixture, 80 mm outboard of socket 1.
        table_stage = right_pick_pose((g.STAGE_X, g.SOCKET_Y[0] - 0.08, g.TOP + g.HOUSING[2] / 2))

        # Pull Green and drop on table
        add("right", 0.7, (pick_s1[0], pick_s1[1], hover_z), finger_release, "r: hover green s1", op, "hover")
        add("right", 0.7, pick_s1, finger_release, "r: descend green s1", op, "descend")
        add("right", 0.5, pick_s1, finger_grip, "r: grip green s1", op, "grip")
        add("right", 0.7, (pull_s1[0], pull_s1[1], pick_s1[2]), finger_grip, "r: pull green s1", op, "pull")
        add("right", 0.8, table_stage, finger_grip, "r: place green on table", op, "place")
        add("right", 0.5, table_stage, finger_open, "r: release green on table", op, "release")
        add("right", 0.7, (table_stage[0], table_stage[1], hover_z), finger_open, "r: lift after drop", op, "lift")
        op += 1

        # Move Red from Socket 2 (index 1) -> Socket 1 (index 0)
        module_transfer(from_sock=1, to_sock=0, mod_name="red", op_idx=op)
        op += 1

        add("right", 0.8, home_r, finger_open, "r: return home", op, "transit")
        sync("after_no_buffer_moves", op)
        op += 1

        # Final hold (Green is on table, Red in s1 matching -> score 0.50)
        add("left", 2.5, home_l, finger_open, "l: final hold", op, "hold")
        add("right", 2.5, home_r, finger_open, "r: final hold", op, "hold")

        out = {"left": left, "right": right}
        _scale_plans(out)
        return (out, meta) if with_meta else out

    # Standard 3-move demo plan:
    # 1. Green module from Socket 1 (index 0) -> Socket 4 (buffer, index 3)
    module_transfer(from_sock=0, to_sock=3, mod_name="green", op_idx=op)
    op += 1

    # 2. Red module from Socket 2 (index 1) -> Socket 1 (matching, index 0)
    module_transfer(from_sock=1, to_sock=0, mod_name="red", op_idx=op)
    op += 1

    # 3. Green module from Socket 4 (index 3) -> Socket 2 (matching, index 1)
    module_transfer(from_sock=3, to_sock=1, mod_name="green", op_idx=op)
    op += 1

    # Right arm returns home
    add("right", 0.8, home_r, finger_open, "r: return home after moves", op, "transit")
    sync("after_moves", op)
    op += 1

    # Phase 3: Left arm presses COMMIT button (unless mode == 'no_commit')
    commit_xy = (g.COMMIT_BTN_POS[0], g.COMMIT_BTN_POS[1])
    if mode != "no_commit":
        add("left", 0.8, (commit_xy[0], commit_xy[1], hover_z), finger_btn, "l: hover COMMIT", op, "hover")
        add("left", 0.8, (commit_xy[0], commit_xy[1], press_z), finger_btn, "l: press COMMIT", op, "commit_press")
        add("left", 0.6, (commit_xy[0], commit_xy[1], press_z), finger_btn, "l: hold COMMIT", op, "commit_press")
        add("left", 0.6, (commit_xy[0], commit_xy[1], hover_z), finger_btn, "l: release COMMIT", op, "lift")
        add("left", 0.8, home_l, finger_open, "l: return home after commit", op, "transit")
        sync("after_commit", op)
        op += 1

    # Phase 4: Final hold at home with open grippers
    hold_s = 2.5
    add("left", hold_s, home_l, finger_open, "l: final hold", op, "hold")
    add("right", hold_s, home_r, finger_open, "r: final hold", op, "hold")

    out = {"left": left, "right": right}
    _scale_plans(out)
    return (out, meta) if with_meta else out


def _scale_plans(seqs: dict[str, list[Waypoint]]) -> None:
    """Scale all waypoint durations by DEMO_TIME_SCALE (default 1.8)."""
    scale = _f("DEMO_TIME_SCALE", 1.8)
    if scale != 1.0:
        for arm in ("left", "right"):
            seqs[arm][:] = [replace(w, duration=w.duration * scale) for w in seqs[arm]]


def plans(mode: str | None = None, with_meta: bool = False):
    return demo_plans(mode=mode, with_meta=with_meta)
