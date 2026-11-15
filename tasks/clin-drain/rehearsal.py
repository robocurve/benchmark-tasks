"""Standalone MuJoCo physics rehearsal of clin-drain with physical finger-pad contacts.

Rebuilds the non-slip mat, 3-spring expandable wound-drain reservoir (two telescoping
Z-axis slide stages with calibrated spring stiffness), hollow flared pouring spout, and
free-body pouring-spout plug directly from clin_drain.py, then drives two dynamic
Cartesian hands carrying physical actuated parallel finger pads (never weld constraints)
along the task's own demo() ArmPlan trajectories.

Usage:
    python rehearsal.py
"""

from __future__ import annotations

import math
import pathlib
import sys
import time

import mujoco
import numpy as np

PKG_DIR = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(PKG_DIR))

import labkit  # noqa: E402
from labkit import PartState  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

DT = 0.001
CONTROL_DT = 0.05
PAD_HALF_X = 0.003   # 6 mm pad thickness along closing axis (local X of ArmPlan rot)
PAD_HALF_Y = 0.008   # 16 mm pad width matching YAM finger pad
PAD_HALF_Z = 0.014   # 28 mm tall, tip at local +Z = +PAD_TIP_PAST_TCP (down in world)
PAD_CENTER_Z = PAD_TIP_PAST_TCP - PAD_HALF_Z


def _quat_wxyz(yaw: float = 0.0, pitch: float = 0.0) -> tuple[float, float, float, float]:
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    return (cy * cp, -sy * sp, cy * sp, sy * cp)


def _axis_quat_wxyz(axis: str) -> str:
    if axis == "X":
        return "0.70710678 0 0.70710678 0"
    if axis == "Y":
        return "0.70710678 -0.70710678 0 0"
    return "1 0 0 0"


def _geoms_xml(boxes, cylinders, mass, mu: float, group: int = 0) -> str:
    out, vol = [], 0.0
    for c, s, k, (yaw, pitch) in boxes:
        q = _quat_wxyz(yaw, pitch)
        out.append(
            f'<geom type="box" pos="{c[0]} {c[1]} {c[2]}" size="{s[0]/2} {s[1]/2} {s[2]/2}" '
            f'quat="{q[0]} {q[1]} {q[2]} {q[3]}" rgba="{k[0]} {k[1]} {k[2]} 1" '
            f'friction="{mu} 0.005 0.0001" group="{group}" DENS/>'
        )
        vol += s[0] * s[1] * s[2]
    for c, r, h, k, axis in cylinders:
        qstr = _axis_quat_wxyz(axis)
        out.append(
            f'<geom type="cylinder" pos="{c[0]} {c[1]} {c[2]}" size="{r} {h/2}" '
            f'quat="{qstr}" rgba="{k[0]} {k[1]} {k[2]} 1" '
            f'friction="{mu} 0.005 0.0001" group="{group}" DENS/>'
        )
        vol += math.pi * r * r * h
    dens = f'density="{mass / max(vol, 1e-9):.9g}"' if mass is not None else ""
    return "\n".join(g.replace("DENS", dens) for g in out)


def _joint_tree_xml(joints_list, base_mu: float) -> tuple[str, list[str]]:
    by_parent: dict[str | None, list] = {}
    for j in joints_list:
        by_parent.setdefault(j.parent, []).append(j)
    actuators: list[str] = []

    def emit(parent_name: str = "") -> list[str]:
        lines = []
        for j in by_parent.get(parent_name, []):
            axis_v = {"X": "1 0 0", "Y": "0 1 0", "Z": "0 0 1"}[j.axis]
            jtype = "hinge" if j.kind == "hinge" else "slide"
            a = j.anchor
            lines.append(f'<body name="j_{j.name}">')
            lines.append(
                f'<joint name="{j.name}" type="{jtype}" pos="{a[0]} {a[1]} {a[2]}" axis="{axis_v}" '
                f'range="{j.lower} {j.upper}" damping="{j.damping:.9g}" frictionloss="{j.friction:.9g}"/>'
            )
            lines.append(_geoms_xml(j.boxes, j.cylinders, j.mass, base_mu))
            actuators.append(
                f'<position name="act_{j.name}" joint="{j.name}" kp="{j.stiffness:.9g}" kv="{j.damping:.9g}"/>'
            )
            lines.extend(emit(j.name))
            lines.append("</body>")
        return lines

    return "\n".join(emit("")), actuators


def _build_xml(task) -> str:
    M = task.module
    bodies = []
    actuators = []
    excludes = []
    for spec in task.objects:
        q = _quat_wxyz(spec.yaw)
        jtag = "<freejoint/>" if spec.mass is not None else ""
        gxml = _geoms_xml(spec.boxes, spec.cylinders, spec.mass, spec.friction)
        jlist = getattr(spec, "joints", ())
        jxml, jacts = _joint_tree_xml(jlist, spec.friction)
        actuators.extend(jacts)
        if jlist:
            part_bodies = [spec.name] + [f"j_{j.name}" for j in jlist]
            for i, b1 in enumerate(part_bodies):
                for b2 in part_bodies[i + 1 :]:
                    excludes.append(f'<exclude body1="{b1}" body2="{b2}"/>')
        bodies.append(
            f'<body name="{spec.name}" pos="{spec.pos[0]} {spec.pos[1]} {spec.pos[2]}" '
            f'quat="{q[0]} {q[1]} {q[2]} {q[3]}">{jtag}\n{gxml}\n{jxml}\n</body>'
        )
    # Dynamic 4-DOF Cartesian hands + physical parallel-jaw slide pads (real contact, zero welds)
    for arm in ("left", "right"):
        bodies.append(
            f'<body name="hand_{arm}" pos="0 0 0">'
            f'  <joint name="{arm}_px" type="slide" axis="1 0 0" range="-0.5 1.0" damping="10"/>'
            f'  <joint name="{arm}_py" type="slide" axis="0 1 0" range="-0.8 0.8" damping="10"/>'
            f'  <joint name="{arm}_pz" type="slide" axis="0 0 1" range="0.5 1.5" damping="10"/>'
            f'  <joint name="{arm}_rz" type="hinge" axis="0 0 1" range="-3.1416 3.1416" damping="1"/>'
            f'  <inertial pos="0 0 0" mass="0.5" diaginertia="0.002 0.002 0.002"/>'
            f'  <body name="wrist_{arm}" pos="0 0 0" quat="0 0.70710678 -0.70710678 0">'
            f'    <inertial pos="0 0 0" mass="0.1" diaginertia="0.0002 0.0002 0.0002"/>'
            f'    <body name="{arm}_pad_pos" pos="{PAD_HALF_X} 0 {PAD_CENTER_Z}">'
            f'      <joint name="{arm}_fpos" type="slide" axis="-1 0 0" range="{FINGER_OPEN} 0.002" damping="2"/>'
            f'      <geom type="box" size="{PAD_HALF_X} {PAD_HALF_Y} {PAD_HALF_Z}" mass="0.05" '
            f'            friction="1.5 0.005 0.0001" rgba="0.18 0.18 0.20 1"/>'
            f'    </body>'
            f'    <body name="{arm}_pad_neg" pos="-{PAD_HALF_X} 0 {PAD_CENTER_Z}">'
            f'      <joint name="{arm}_fneg" type="slide" axis="1 0 0" range="{FINGER_OPEN} 0.002" damping="2"/>'
            f'      <geom type="box" size="{PAD_HALF_X} {PAD_HALF_Y} {PAD_HALF_Z}" mass="0.05" '
            f'            friction="1.5 0.005 0.0001" rgba="0.18 0.18 0.20 1"/>'
            f'    </body>'
            f'  </body>'
            f'</body>'
        )
        actuators.extend([
            f'<position name="act_{arm}_px" joint="{arm}_px" kp="8000" kv="160" forcerange="-80 80"/>',
            f'<position name="act_{arm}_py" joint="{arm}_py" kp="8000" kv="160" forcerange="-80 80"/>',
            f'<position name="act_{arm}_pz" joint="{arm}_pz" kp="8000" kv="160" forcerange="-80 80"/>',
            f'<position name="act_{arm}_rz" joint="{arm}_rz" kp="250" kv="12"/>',
            f'<position name="act_{arm}_fpos" joint="{arm}_fpos" kp="5000" kv="80" forcerange="-35 35"/>',
            f'<position name="act_{arm}_fneg" joint="{arm}_fneg" kp="5000" kv="80" forcerange="-35 35"/>',
        ])

    xml = f"""<mujoco model="clin_drain_rehearsal">
<compiler angle="radian" autolimits="true" boundmass="1e-7" boundinertia="1e-12"/>
<option timestep="{DT}" integrator="implicitfast" cone="elliptic" impratio="100" noslip_iterations="0">
  <flag multiccd="enable"/>
</option>
<default><geom condim="4" solref="0.008 1" solimp="0.95 0.99 0.0005"/></default>
<worldbody>
  <geom name="table" type="box" pos="0.30 0 {M.TOP - 0.03}" size="0.60 0.65 0.03"
        friction="0.6 0.005 0.0001" rgba="0.82 0.62 0.36 1"/>
  {chr(10).join(bodies)}
</worldbody>
<contact>
  {chr(10).join(excludes)}
</contact>
<actuator>
  {chr(10).join(actuators)}
</actuator>
</mujoco>"""
    return xml


def run() -> tuple[float, bool]:
    t_wall = time.perf_counter()
    task = load(PKG_DIR / "clin_drain.py")
    M = task.module
    specs = {o.name: o for o in task.objects}
    model = mujoco.MjModel.from_xml_string(_build_xml(task))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    plans = {arm: ArmPlan(wps, station.BASES[arm]) for arm, wps in task.demo().items()}
    total_s = max(p.duration for p in plans.values())

    act_ids = {}
    jnt_ids = {}
    for arm in ("left", "right"):
        act_ids[arm] = tuple(
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{arm}_{k}")
            for k in ("px", "py", "pz", "rz", "fpos", "fneg")
        )
        jnt_ids[arm] = tuple(
            model.jnt_qposadr[model.joint(f"{arm}_{k}").id]
            for k in ("px", "py", "pz", "rz", "fpos", "fneg")
        )
    drain_acts = tuple(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{j.name}")
        for j in specs["drain"].joints
    )
    joint_names = {o.name: [j.name for j in getattr(o, "joints", ())] for o in task.objects}
    extents = {o.name: labkit._extent(o.boxes, o.cylinders) for o in task.objects}

    def read_states() -> dict[str, PartState]:
        out = {}
        for name, spec in specs.items():
            bid = model.body(name).id
            pos = data.xpos[bid].copy()
            w, x, y, z = data.xquat[bid]
            lin = ang = np.zeros(3)
            if spec.mass is not None:
                adr = model.jnt_dofadr[model.body_jntadr[bid]]
                lin, ang = data.qvel[adr : adr + 3].copy(), data.qvel[adr + 3 : adr + 6].copy()
            jvals = {jn: float(data.qpos[model.jnt_qposadr[model.joint(jn).id]]) for jn in joint_names[name]}
            lo, hi = extents[name]
            probe = PartState(pos, np.array([x, y, z, w]), lin, ang, np.zeros(3), np.zeros(3), jvals)
            pts = np.array([probe.to_world((a, b, c)) for a in (lo[0], hi[0]) for b in (lo[1], hi[1]) for c in (lo[2], hi[2])])
            out[name] = PartState(pos, probe.quat_xyzw, lin, ang, pts.min(0), pts.max(0), jvals)
        return out

    def hand_tcps() -> tuple[np.ndarray, np.ndarray]:
        out = []
        for arm in ("left", "right"):
            px, py, pz = jnt_ids[arm][:3]
            out.append(np.array([float(data.qpos[px]), float(data.qpos[py]), float(data.qpos[pz])], float))
        return (out[0], out[1])

    def cmd_arm(arm: str, pos, rot, finger: float, snap: bool = False):
        yaw = float(math.atan2(rot[0, 0], -rot[1, 0]))
        vals = (float(pos[0]), float(pos[1]), float(pos[2]), yaw, float(finger), float(finger))
        for aid, v in zip(act_ids[arm], vals):
            data.ctrl[aid] = v
        if snap:
            for qadr, v in zip(jnt_ids[arm], vals):
                data.qpos[qadr] = v

    def watch_step() -> None:
        st = read_states()
        drain, plug = st["drain"], st["plug"]
        M._trial["tcps"] = hand_tcps()
        h = M._drain_height_m(drain)
        if float(drain.pos[2]) > M.MAT_TOP + 0.025:
            M._trial["lifted_off_mat"] = True
            M._trial["compressed_ever"] = False
        elif (
            h <= M.COMPRESSED_MAX_H
            and M._tilt_deg(drain) <= M.DRAIN_TILT_MAX_DEG
            and M._on_mat(drain)
            and float(drain.pos[2]) <= M.MAT_TOP + 0.012
            and not M._trial["lifted_off_mat"]
        ):
            M._trial["compressed_ever"] = True

        seated_now = M._plug_in_spout(drain, plug)
        if seated_now:
            if not M._trial["sealed"]:
                M._trial["sealed"] = True
                qm = float(np.clip(drain.joints.get("bellows_mid", 0.0), -M.STROKE_EACH, 0.0))
                qt = float(np.clip(drain.joints.get("top_plate", 0.0), -M.STROKE_EACH, 0.0))
                M._trial["sealed_q"] = (qm, qt)
                if h > M.COMPRESSED_MAX_H:
                    M._trial["sealed_while_expanded"] = True
        else:
            M._trial["sealed"] = False
            M._trial["sealed_q"] = (0.0, 0.0)

        if (
            seated_now
            and not M._trial["sealed_while_expanded"]
            and h <= M.HELD_MAX_H
            and M._tilt_deg(drain) <= M.DRAIN_TILT_MAX_DEG
            and M._on_mat(drain)
            and M._hands_clear(drain, plug, M._trial["tcps"])
            and drain.resting()
            and plug.resting()
        ):
            M._trial["clear_s"] += CONTROL_DT
        else:
            M._trial["clear_s"] = 0.0

        tgt = M._trial["sealed_q"] if M._trial["sealed"] else (0.0, 0.0)
        for aid, qval in zip(drain_acts, tgt):
            data.ctrl[aid] = float(qval)

    M._reset(None)
    for arm, plan in plans.items():
        pos0, rot0, f0, _ = plan.target(0.0)
        cmd_arm(arm, pos0, rot0, f0, snap=True)
    for aid in drain_acts:
        data.ctrl[aid] = 0.0
    mujoco.mj_forward(model, data)

    for _ in range(round(0.25 / DT)):
        mujoco.mj_step(model, data)

    steps_per_ctl = round(CONTROL_DT / DT)
    total_ctl = math.ceil((total_s + 0.5) / CONTROL_DT)
    for k in range(total_ctl):
        t0 = k * CONTROL_DT
        for s in range(steps_per_ctl):
            t_sub = t0 + (s + 1) * DT
            for arm, plan in plans.items():
                pos, rot, finger, _ = plan.target(min(t_sub, plan.duration))
                cmd_arm(arm, pos, rot, finger)
            mujoco.mj_step(model, data)
        watch_step()

    final_st = read_states()
    sc = float(task.score(final_st))
    ok = bool(task.success(final_st))
    wall = time.perf_counter() - t_wall
    d, pl = final_st["drain"], final_st["plug"]
    print(
        f"drain: h={M._drain_height_m(d)*1000:.1f}mm ({M._drain_height_m(d)/M.H_EXPANDED*100:.1f}%) "
        f"tilt={M._tilt_deg(d):.2f}deg resting={d.resting()}"
    )
    print(
        f"plug: pos=({pl.pos[0]:.4f}, {pl.pos[1]:.4f}, {pl.pos[2]:.4f}) "
        f"in_spout={M._plug_in_spout(d, pl)} tilt={M._tilt_deg(pl):.2f}deg resting={pl.resting()}"
    )
    print(
        f"trial: compressed_ever={M._trial['compressed_ever']} sealed={M._trial['sealed']} "
        f"sealed_while_expanded={M._trial['sealed_while_expanded']} clear_s={M._trial['clear_s']:.2f} "
        f"hands_clear={M._hands_clear(d, pl, M._trial['tcps'])}"
    )
    print(f"wall_s={wall:.2f} SCORE {sc} SUCCESS {ok}")
    return sc, ok


if __name__ == "__main__":
    score, ok = run()
    sys.exit(0 if ok and score >= 1.0 - 1e-9 else 1)
