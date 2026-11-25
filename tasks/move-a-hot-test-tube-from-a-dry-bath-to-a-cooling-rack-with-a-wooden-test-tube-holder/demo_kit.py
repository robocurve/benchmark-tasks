"""Scripted arm motions for a ``yam_arena`` task file: the runner behind a task's demo and its mechanics trials.

A motion is a list of :class:`Move` per arm: where the gripper's grasp point goes, how the gripper is turned, how
far it is open, and in how many seconds. Positions are blended smoothly, rotations along the shortest turn, and
every control step the pose is turned into joint targets with yam_arena's IK on the arm's URDF. A move may also
press with a given force (``push``). The plan is open loop: it never reads the simulator; what a run reads (the
arms' joints, for an estimate of the force they press with) goes into its record only.

Runs inside the simulator container:

    scripts/rta simpy <id> <script.py> ...      # a script that builds moves and calls run()

``run`` steps the stock embodiment, records what the kit and the task report each step, and writes into the output
folder: ``result.json``, one MP4 per camera, ``setup.png`` (the top camera before the first step) and a contact
sheet per camera (``sheet_<camera>.png``: frames at even intervals, for a look without a video player).
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

CAMERAS = ("top_cam", "left_cam", "right_cam")
SIDES = ("left", "right")
POLICY_PICTURE = (320, 176)  # width, height of the pictures a policy gets in the program's model runs
JOINT_STIFFNESS = np.array([80.0, 80.0, 80.0, 10.0, 10.0, 10.0])  # N m/rad of joints 1-6: yam_arena.constants SHOULDER and WRIST
JOINT_EFFORT = np.array([27.0, 27.0, 27.0, 7.0, 7.0, 7.0])  # N m, the most each joint's drive gives (same constants)


@dataclass
class Move:
    """Reach ``pos`` (grasp point, world, m) with the gripper turned as ``rot`` (3x3, its columns are the closing
    axis, the pads' width axis and the approach axis) and opened to ``grip`` (0 closed .. 1 open) after
    ``seconds``. None keeps what the arm has.

    ``push``: a force (N, world) to press with, for a gripper that already touches something. The arm is driven by
    position: a joint held away from its target pushes back like a spring (:data:`JOINT_STIFFNESS`), so a force F
    at a point of the gripper is asked for as the joint offsets J^T F / stiffness on top of the pose's joints (J:
    that point's Jacobian). While the touching point cannot move, the joints stay where the pose put them and the
    point presses with F and nothing else. A target simply moved "into" the surface presses too, but also
    sideways: the arm is not equally stiff in all directions, and a light free-standing object is shoved away (run
    n2e: 25 mm down gave 14 N down and 16 N sideways by this same reckoning, and moved a cartridge 18.5 mm).
    ``push_at``: the touching point, from the grasp point in the gripper's frame (m); see :func:`lowest_point`."""

    seconds: float
    pos: Any = None
    rot: Any = None
    grip: float | None = None
    label: str = ""
    push: Any = None  # None keeps the last (no push at first); blended over the move like the pose
    push_at: Any = None  # None keeps the last (the grasp point at first); taken at the move's start


def down(yaw: float = 0.0) -> np.ndarray:
    """The gripper pointing straight down, turned ``yaw`` about the vertical (0: the fingers close along world y)."""
    from yam_arena.demo import grip_rotation

    return grip_rotation(yaw)


def about(axis, angle: float) -> np.ndarray:
    """Rotation by ``angle`` rad about ``axis`` (world)."""
    from yam_arena.urdf import axis_angle_matrix

    a = np.asarray(axis, dtype=float)
    return axis_angle_matrix(a / np.linalg.norm(a), angle)


_HAND: list[np.ndarray] = []  # the closed hand's collision corners from the grasp point in the gripper's frame, read once


def lowest_point(rot, band: float = 0.0007) -> np.ndarray:
    """Where the closed hand is lowest when turned as ``rot``: the middle of the corners of its collision hulls
    (gripper and both tips, from the URDF's meshes) that lie within ``band`` of the lowest one, from the grasp
    point in the gripper's frame (m). That is where it touches a level surface it is lowered onto. Straight down
    it is the middle of the tips' ends, 13 mm past the grasp point; leaning, the lower tip's end edge."""
    from yam_arena import analysis
    from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_JOINTS, READY, TCP_OFFSET

    if not _HAND:
        joints = {**{n: float(v) for n, v in zip(ARM_JOINTS, READY)}, **dict.fromkeys(FINGER_JOINTS, FINGER_CLOSED)}
        poses = analysis.fk_link_poses_world(joints, (0.0, 0.0, 0.0))
        gripper = poses["gripper"]
        corners = np.vstack([hull.vertices for _, hull in analysis.world_hulls(poses, ["gripper", "tip_left", "tip_right"])])
        _HAND.append((corners - gripper[:3, 3]) @ gripper[:3, :3] - np.array(TCP_OFFSET))
    height = _HAND[0] @ np.asarray(rot, dtype=float)[2]  # each corner's height in the world, from the grasp point
    return _HAND[0][height < height.min() + band].mean(axis=0)


def point_jacobian(q: np.ndarray, base, at=(0.0, 0.0, 0.0)) -> np.ndarray:
    """3x6: how the point ``at`` (from the grasp point, in the gripper's frame, m) moves in the world per radian of
    each arm joint, at joints ``q``."""
    from yam_arena import ik

    at = np.asarray(at, dtype=float)

    def point(joints: np.ndarray) -> np.ndarray:
        pos, rot = ik.tcp_pose(joints, base)
        return pos + rot @ at

    eps, here = 1e-6, point(np.asarray(q, dtype=float))
    return np.column_stack([(point(q + eps * np.eye(6)[i]) - here) / eps for i in range(6)])


def spring_force(sent: np.ndarray, seen: np.ndarray, base, at=(0.0, 0.0, 0.0)) -> np.ndarray:
    """An estimate of the force (N, world) an arm at rest puts on what it touches at the point ``at``: the pure
    force there whose joint torques come closest to the drives' spring torques, stiffness x (joint targets ``sent``
    - joints ``seen``). It leaves out joint friction and what is left of gravity, and means nothing while the arm
    moves."""
    torque = JOINT_STIFFNESS * (np.asarray(sent, dtype=float)[:6] - np.asarray(seen, dtype=float)[:6])
    return np.linalg.lstsq(point_jacobian(np.asarray(seen, dtype=float)[:6], base, at).T, torque, rcond=None)[0]


class Arm:
    """One arm following its moves; ``action(t)`` is its [j1..j6, grip] at time t."""

    def __init__(self, side: str, base, moves: list[Move]) -> None:
        from yam_arena import ik
        from yam_arena.constants import READY

        self.side, self.base, self.moves = side, np.asarray(base, dtype=float), list(moves)
        self.q = np.array(READY, dtype=float)
        self.start_pos, self.start_rot = ik.tcp_pose(self.q, self.base)
        self.worst_mm, self.worst_deg, self.worst_effort = 0.0, 0.0, 0.0  # worst_effort: a push's joint torque / the drive's most

    @property
    def seconds(self) -> float:
        return sum(m.seconds for m in self.moves)

    def target(self, t: float) -> tuple[np.ndarray, np.ndarray, float, str]:
        return self.state(t)[:4]

    def state(self, t: float) -> tuple[np.ndarray, np.ndarray, float, str, np.ndarray, np.ndarray]:
        """(grasp point, rotation, grip, label, push, the point it pushes with) at time t."""
        from yam_arena import ik

        pos, rot, grip, push, at = self.start_pos, self.start_rot, 1.0, np.zeros(3), np.zeros(3)
        for move in self.moves:
            goal_pos = pos if move.pos is None else np.asarray(move.pos, dtype=float)
            goal_rot = rot if move.rot is None else np.asarray(move.rot, dtype=float)
            goal_grip = grip if move.grip is None else float(move.grip)
            goal_push = push if move.push is None else np.asarray(move.push, dtype=float)
            at = at if move.push_at is None else np.asarray(move.push_at, dtype=float)
            if t < move.seconds:
                s = ik.smoothstep(t / move.seconds)
                return pos + s * (goal_pos - pos), ik.slerp_matrix(rot, goal_rot, s), grip + s * (goal_grip - grip), move.label, push + s * (goal_push - push), at
            t -= move.seconds
            pos, rot, grip, push = goal_pos, goal_rot, goal_grip, goal_push
        return pos, rot, grip, "done", push, at

    def press_offsets(self, q: np.ndarray, push: np.ndarray, at=(0.0, 0.0, 0.0)) -> np.ndarray:
        """The joint offsets that make the position-driven arm press with ``push`` (N, world) at its point ``at``:
        J^T F / stiffness. A push that asks a joint for more than its drive gives is refused: the arm would press
        with some other force, in some other direction."""
        torque = point_jacobian(q, self.base, at).T @ np.asarray(push, dtype=float)
        self.worst_effort = max(self.worst_effort, float(np.abs(torque / JOINT_EFFORT).max()))
        if self.worst_effort > 1.0:
            raise ValueError(f"{self.side} arm: a push of {np.asarray(push).round(1).tolist()} N needs joint torques {torque.round(2).tolist()} N m, beyond the drives' {JOINT_EFFORT.tolist()}")
        return torque / JOINT_STIFFNESS

    def action(self, t: float) -> tuple[np.ndarray, str]:
        from yam_arena import ik

        pos, rot, grip, label, push, at = self.state(t)
        q, pos_err, rot_err = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q = q
        self.worst_mm, self.worst_deg = max(self.worst_mm, pos_err * 1000.0), max(self.worst_deg, math.degrees(rot_err))
        if np.any(push != 0.0):
            pressed = q + self.press_offsets(q, push, at)
            if np.any(pressed < ik.LOWER) or np.any(pressed > ik.UPPER):
                raise ValueError(f"{self.side} arm: the push at {label!r} needs joint targets {np.degrees(pressed).round(1).tolist()} deg, beyond a joint's limit")
            q = pressed
        return np.r_[q, grip], label


def dry_run(moves: dict[str, list[Move]], bases: dict, director: Any = None, hz: float = 20.0, tail_s: float = 1.0, limit_s: float = 120.0) -> dict:
    """The plan without the simulator: every pose through the IK. Returns per arm the worst IK error, the smallest
    distance to a joint limit and where it occurs, and the plan's length in seconds. The director gets no
    pictures (None)."""
    from yam_arena import ik

    arms = {side: Arm(side, bases[side], moves.get(side, [])) for side in SIDES}
    tight = {side: (1e9, None) for side in SIDES}
    i = 0
    while True:
        t = i / hz
        if director is not None:
            director(t, arms, None)
        if t >= min(limit_s, max(arm.seconds for arm in arms.values()) + tail_s):
            break
        for side, arm in arms.items():
            action, label = arm.action(t)
            margin = float(np.degrees(np.minimum(action[:6] - ik.LOWER, ik.UPPER - action[:6])).min())
            if margin < tight[side][0]:
                tight[side] = (margin, f"{label} at {t:.2f} s, joints {np.degrees(action[:6]).round(0).astype(int).tolist()}")
        i += 1
    return {side: {"seconds": round(arm.seconds, 2), "worst_ik_mm": round(arm.worst_mm, 2), "worst_ik_deg": round(arm.worst_deg, 2),
                   "smallest_margin_deg": round(tight[side][0], 1), "there": tight[side][1], "push_effort": round(arm.worst_effort, 2)} for side, arm in arms.items()}  # fmt: skip


def policy_picture(frame: np.ndarray, size: tuple[int, int] = POLICY_PICTURE) -> np.ndarray:
    """A camera frame as a policy gets it at ``size`` (width, height): yam_arena renders 640 x 360 and resizes with
    this same call (inspect_embodiment.YamArenaEmbodiment)."""
    import cv2

    return np.ascontiguousarray(cv2.resize(frame, size), dtype=np.uint8)


def disc_mask(camera, shape: tuple[int, int], centre, radius: float) -> np.ndarray:
    """The pixels of a picture of ``shape`` (rows, columns) whose line of sight meets the level disc of ``radius``
    about ``centre`` (world, m): where a camera sees a round, level surface, if nothing stands in front of it.
    ``camera``: a rig camera fixed in the world (``embodiment.rig.top_cam``); its lens is rescaled to the picture,
    as the embodiment's resize does."""
    from yam_arena.urdf import quat_xyzw_to_matrix

    if camera.convention != "opengl":
        raise ValueError(f"disc_mask knows the opengl camera frame (x right, y up, looking along -z), not {camera.convention!r}")
    rows, columns = shape
    lens = camera.lens.resized(columns, rows)
    pos, rot = np.array(camera.pos, dtype=float), quat_xyzw_to_matrix(np.array(camera.rot_xyzw, dtype=float))
    u, v = np.meshgrid(np.arange(columns) + 0.5, np.arange(rows) + 0.5)
    rays = np.stack([(u - lens.cx) / lens.fx, -(v - lens.cy) / lens.fy, -np.ones_like(u)], axis=-1) @ rot.T  # in the world
    centre = np.asarray(centre, dtype=float)
    with np.errstate(divide="ignore", invalid="ignore"):
        along = (centre[2] - pos[2]) / rays[..., 2]
    hit = pos[:2] + along[..., None] * rays[..., :2] - centre[:2]
    return (along > 0) & (np.hypot(hit[..., 0], hit[..., 1]) <= radius)


def contact_sheet(frames: list[np.ndarray], labels: list[str], columns: int = 4, width: int = 320) -> np.ndarray:
    """Frames side by side in rows, each with its label, as one picture (BGR)."""
    import cv2

    tiles = []
    for frame, label in zip(frames, labels):
        h = int(round(frame.shape[0] * width / frame.shape[1]))
        tile = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR), (width, h), interpolation=cv2.INTER_AREA)
        cv2.rectangle(tile, (0, 0), (width, 18), (0, 0, 0), -1)
        cv2.putText(tile, label, (4, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        tiles.append(tile)
    while len(tiles) % columns:
        tiles.append(np.zeros_like(tiles[0]))
    return np.vstack([np.hstack(tiles[i : i + columns]) for i in range(0, len(tiles), columns)])


def open_task(task: str | Path, hz: float = 20.0):
    """The stock embodiment on a task file, as a demo runs it: fixed layout, 640x360 pictures. One per process."""
    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    return YamArenaEmbodiment(task=str(task), object_jitter=0.0, control_hz=hz, cam_width=640, cam_height=360)


def run(embodiment, moves: dict[str, list[Move]], out: str | Path, *, seed: int | None = 0, director: Any = None, tail_s: float = 1.0,
        limit_s: float = 120.0, sheet_frames: int = 12, crop: dict | None = None, extra: Any = None, record: bool = True) -> dict:  # fmt: skip
    """Reset ``embodiment`` (from :func:`open_task`) with ``seed``, play ``moves`` (side -> list of Move; a missing
    side holds the ready pose) and write the results into ``out``. It ends ``tail_s`` after the last move, at
    ``limit_s``, or when the task ends the episode.

    ``director(t, arms, pictures)``, if given, is called before every step with the cameras' latest pictures
    (name -> RGB array; None in a dry run) and may append moves to ``arms[side].moves``: that is how a demo acts on
    what it sees. A decision that depends on a hidden state is taken from those pictures, not from the simulator.
    ``crop``: camera -> (x0, y0, x1, y1) in 640x360 pixels for that camera's contact sheet. ``extra()`` is called
    every step and its result stored in the step's row. Several runs may follow each other on one embodiment.
    ``record=False``: no videos (a check, not a recording)."""
    import cv2
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station

    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    hz = float(embodiment.info.control_hz)
    task = embodiment.task.path
    import mechkit  # the task's own copy: the task file put its folder on sys.path

    module = embodiment.task.module
    bases = station.BASES if embodiment.scene == "station" else None
    if bases is None:
        from yam_arena.constants import LEFT_BASE_POS, RIGHT_BASE_POS

        bases = {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}
    arms = {side: Arm(side, bases[side], moves.get(side, [])) for side in SIDES}
    observation = embodiment.reset(Scene(id="demo", instruction=getattr(module, "INSTRUCTION", "")), seed=seed)
    cv2.imwrite(str(out / "setup.png"), cv2.cvtColor(observation.images["top_cam"], cv2.COLOR_RGB2BGR))
    import imageio  # its ffmpeg plugin writes H.264; this OpenCV build has no MP4 encoder (its writer fails without a word)

    writers = {cam: imageio.get_writer(str(out / f"{cam}.mp4"), fps=hz, codec="libx264", quality=7, macro_block_size=1) for cam in CAMERAS} if record else {}
    kept: dict[str, list] = {cam: [] for cam in CAMERAS}
    kept_labels: list[str] = []
    rows, result, spent, i = [], None, [], 0
    try:
        while True:
            t = i / hz
            if director is not None:
                director(t, arms, observation.images)
            if t >= min(limit_s, max(arm.seconds for arm in arms.values()) + tail_s):
                break
            left, left_label = arms["left"].action(t)
            right, right_label = arms["right"].action(t)
            start = time.perf_counter()
            result = embodiment.step(Action(data=pack(left, right)))
            spent.append((time.perf_counter() - start) * 1000.0)
            observation = result.observation
            for cam, writer in writers.items():
                writer.append_data(observation.images[cam])
            seen = np.asarray(next(iter(observation.state.values())), dtype=float).reshape(-1)  # left 7 then right 7, as sent
            row = {
                "t": round((i + 1) / hz, 3),
                "left": left_label,
                "right": right_label,
                # What each arm's joint springs press with at the point it pushes with (the grasp point otherwise): an estimate, good at rest only.
                "force": {side: [round(float(f), 2) for f in spring_force(sent, seen[7 * k : 7 * k + 6], bases[side], arms[side].state(t)[5])] for k, (side, sent) in enumerate((("left", left), ("right", right)))},
                "travel": {k: round(float(v), 5) for k, v in mechkit.MECH.travel.items()},
                "info": {k: (bool(v) if isinstance(v, (bool, np.bool_)) else float(v)) for k, v in result.info.items()},
                "grip": {side: [round(float(c), 4) for c in mechkit.MECH.grippers[side][0]] + [round(mechkit.MECH.grippers[side][1], 3)] for side in SIDES if side in mechkit.MECH.grippers},
            }  # fmt: skip
            if extra is not None:
                row["extra"] = json.loads(json.dumps(extra(), default=str))
            rows.append(row)
            if i % max(1, int(round(hz / 2))) == 0:  # two a second, thinned to the sheet's size at the end
                for cam in CAMERAS:
                    frame = observation.images[cam]
                    if crop and cam in crop:
                        x0, y0, x1, y1 = crop[cam]
                        frame = frame[y0:y1, x0:x1]
                    kept[cam].append(np.ascontiguousarray(frame))
                kept_labels.append(f"{row['t']:.1f}s L:{left_label} R:{right_label}"[:44])
            if i % int(hz) == int(hz) - 1:
                print(f"t {row['t']:6.2f}  L {left_label:<16} R {right_label:<16} travel {row['travel']}  info {row['info']}", flush=True)
            i += 1
            if result.terminated:
                break
    finally:  # closed even when a step raises: what was recorded up to it can be played
        for writer in writers.values():
            writer.close()
    picks = sorted({int(round(k * (len(kept_labels) - 1) / max(1, sheet_frames - 1))) for k in range(sheet_frames)}) if kept_labels else []
    for cam in CAMERAS:
        if picks:
            cv2.imwrite(str(out / f"sheet_{cam.replace('_cam', '')}.png"), contact_sheet([kept[cam][k] for k in picks], [kept_labels[k] for k in picks]))
    report = getattr(module, "report", None)
    summary = {
        "task": str(task),
        "seed": seed,
        "steps": len(rows),
        "seconds": rows[-1]["t"] if rows else 0.0,
        "terminated": bool(result is not None and result.terminated),
        "termination_reason": None if result is None else result.termination_reason,
        "info": rows[-1]["info"] if rows else {},
        "report": json.loads(json.dumps(report(), default=str)) if callable(report) else None,
        "worst_ik": {side: {"mm": round(arm.worst_mm, 2), "deg": round(arm.worst_deg, 2), "push_effort": round(arm.worst_effort, 2)} for side, arm in arms.items()},
        "median_step_ms": round(sorted(spent)[len(spent) // 2], 1) if spent else None,
    }
    (out / "result.json").write_text(json.dumps({**summary, "rows": rows}, indent=1))
    print("result " + json.dumps(summary), flush=True)
    return {**summary, "rows": rows}
