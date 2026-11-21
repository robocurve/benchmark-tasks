"""Measure threading, slot occupancy and independent removal on copies of a saved state."""

from hashlib import sha256

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from scene import NAMES, PITCH, PROFILE, RIM, SLOTS, TRAY_LENGTH, TRAY_X

STATE = mujoco.mjtState.mjSTATE_INTEGRATION
Z = np.array([0.0, 0.0, 1.0])


def snapshot(model, data):
    values = np.empty(mujoco.mj_stateSize(model, STATE))
    mujoco.mj_getState(model, data, values, STATE)
    return values


def restore(model, values):
    trial = mujoco.MjData(model)
    mujoco.mj_setState(model, trial, values, STATE)
    mujoco.mj_forward(model, trial)
    return trial


def state_id(values):
    return sha256(values.astype("<f8", copy=False).tobytes()).hexdigest()


def world_points(data, body, local):
    ref = data.body(body)
    return np.asarray(local) @ ref.xmat.reshape(3, 3).T + ref.xpos


def wire_points(data):
    # Longitudes are ordered around the whisk. Opposite longitudes form an actual closed hoop.
    return np.array(
        [
            [data.site(f"whisk_{loop}_{side}_p{i}").xpos for i in range(len(PROFILE))]
            for side in range(2)
            for loop in range(4)
        ]
    )


def quads_to_triangles(a, b):
    return np.concatenate(
        (np.stack((a[:-1], b[:-1], a[1:]), axis=1), np.stack((a[1:], b[:-1], b[1:]), axis=1))
    )


def pierced(segments, triangles):
    """Finite segment/triangle intersection by barycentric coordinates."""
    start = segments[:, 0, None, :]
    direction = segments[:, 1, None, :] - start
    a, b, c = triangles.transpose(1, 0, 2)
    e1, e2 = b - a, c - a
    h = np.cross(direction, e2)
    det = (e1 * h).sum(axis=-1)
    valid = abs(det) > 1e-12
    denom = np.where(valid, det, 1)
    s = start - a
    u = (s * h).sum(axis=-1) / denom
    q = np.cross(s, e1)
    v = (direction * q).sum(axis=-1) / denom
    t = (e2 * q).sum(axis=-1) / denom
    return bool(np.any(valid & (u >= 0) & (v >= 0) & (u + v <= 1) & (t >= 0) & (t <= 1)))


def tool_segments(data, name):
    if name == "spoon":
        # Follow the shaft, then sample the widening bowl so reversed insertion also counts.
        line = world_points(
            data, name, [(-0.077, 0, 0.006), (-0.039, 0, 0), (0.139, 0, 0), (0.216, 0, 0.002)]
        )
        bowl = world_points(
            data,
            name,
            [
                (0.139, 0, 0),
                (0.177, 0.024, 0.004),
                (0.215, 0, 0.002),
                (0.177, -0.024, 0.004),
                (0.139, 0, 0),
            ],
        )
        return np.concatenate((np.stack((line[:-1], line[1:]), 1), np.stack((bowl[:-1], bowl[1:]), 1)))
    spans = [world_points(data, name, [(-0.213, 0, 0), (0, 0, 0)])]
    for i, (length, w0, w1) in enumerate(
        ((0.023, 0.009, 0.023), (0.027, 0.023, 0.022), (0.025, 0.022, 0.019))
    ):
        for side in (-1, 0, 1):
            spans.append(
                world_points(data, f"spatula_blade_{i}", [(0, side * w0, 0), (length, side * w1, 0)])
            )
        spans.append(world_points(data, f"spatula_blade_{i}", [(length, -w1, 0), (length, w1, 0)]))
    return np.array(spans)


def engagement(data):
    wires = wire_points(data)
    hoops = [quads_to_triangles(wires[i], wires[i + 4]) for i in range(4)]
    gaps = [quads_to_triangles(wires[i], wires[(i + 1) % 8]) for i in range(8)]
    result = {}
    for name in ("spoon", "spatula"):
        spans = tool_segments(data, name)
        pierced_hoops = [i for i, surface in enumerate(hoops) if pierced(spans, surface)]
        crossed_gaps = [i for i, surface in enumerate(gaps) if pierced(spans, surface)]
        result[name] = {
            "caught": bool(pierced_hoops or crossed_gaps),
            "pierced_loops": pierced_hoops,
            "crossed_gaps": crossed_gaps,
        }
    return result


def whisk_shape(data):
    wires = wire_points(data)
    gaps = np.linalg.norm(wires[:4, -1] - wires[4:, -1], axis=1)
    # Check four opposed diameters at the belly and each physical crown closure.
    diameters = np.linalg.norm(wires[:4, 4] - wires[4:, 4], axis=1)
    bend = max(
        abs(data.joint(f"whisk_{i}_{s}_{j}").qpos.item())
        for i in range(4)
        for s in (0, 1)
        for j in ("root", "bend")
    )
    blade = max(abs(data.joint(f"spatula_blade_{i}_bend").qpos.item()) for i in range(3))
    return dict(
        intact=bool(gaps.max() < 0.002 and np.all((diameters >= 0.052) & (diameters <= 0.097))),
        max_crown_gap_mm=round(1000 * gaps.max(), 3),
        belly_diameters_mm=np.round(1000 * diameters, 2).tolist(),
        max_wire_bend_deg=round(np.rad2deg(bend), 2),
        max_blade_bend_deg=round(np.rad2deg(blade), 2),
    )


def contacts(model, data):
    readings = {n: dict(floor_load_N=0.0, robot_contact=False, utensil_contact=False) for n in NAMES}
    wrench = np.empty(6)
    for i, contact in enumerate(data.contact):
        if contact.dist > 1e-6:
            continue
        names = [model.geom(int(g)).name or "" for g in contact.geom]
        owners = [name.partition("_")[0] for name in names]
        if not any(owner in NAMES for owner in owners):
            continue
        mujoco.mj_contactForce(model, data, i, wrench)
        if wrench[0] <= 1e-5:
            continue
        fz = contact.frame.reshape(3, 3)[:, 2] @ wrench[:3]
        for j, owner in enumerate(owners):
            if owner not in readings:
                continue
            other = 1 - j
            if names[other] == "tray_floor":
                readings[owner]["floor_load_N"] += float(fz if j else -fz)
            if owners[other] in ("left", "right"):
                readings[owner]["robot_contact"] = True
            if owners[other] in NAMES and owners[other] != owner:
                readings[owner]["utensil_contact"] = True
    return readings


def extents(model, data, name):
    """Exact world bounds of the compound convex meshes, capsules and cylinder."""
    lows, highs = [], []
    for g in range(model.ngeom):
        if not (model.geom(g).name or "").startswith(name + "_"):
            continue
        pos, rot = data.geom_xpos[g], data.geom_xmat[g].reshape(3, 3)
        size = model.geom_size[g]
        kind = model.geom_type[g]
        if kind == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = model.geom_dataid[g]
            start = model.mesh_vertadr[mesh]
            local = model.mesh_vert[start : start + model.mesh_vertnum[mesh]]
            points = local @ rot.T + pos
            lows.append(points.min(axis=0))
            highs.append(points.max(axis=0))
        else:
            z = rot[:, 2]
            radius = (
                size[0]
                if kind == mujoco.mjtGeom.mjGEOM_CAPSULE
                else size[0] * np.sqrt(np.maximum(0, 1 - z * z))
            )
            span = abs(z) * size[1] + radius
            lows.append(pos - span)
            highs.append(pos + span)
    return np.min(lows, axis=0), np.max(highs, axis=0)


def slot_readings(model, data):
    load = contacts(model, data)
    result = {}
    for name in NAMES:
        low, high = extents(model, data, name)
        weight = model.body_subtreemass[model.body(name).id] * 9.81
        contact = load[name]
        slot = None
        for i, y in enumerate(SLOTS):
            # A 1 mm ruler tolerance for contact against an inner wall.
            if (
                low[0] >= TRAY_X - TRAY_LENGTH / 2 + 0.006 - 0.001
                and high[0] <= TRAY_X + TRAY_LENGTH / 2 - 0.006 + 0.001
                and low[1] >= y - PITCH / 2 + 0.003 - 0.001
                and high[1] <= y + PITCH / 2 - 0.003 + 0.001
            ):
                slot = i + 1
        supported = contact["floor_load_N"] >= 0.80 * weight
        result[name] = dict(
            slot=slot,
            floor_load_N=round(contact["floor_load_N"], 4),
            weight_N=round(weight, 4),
            supported=bool(supported),
            released=not contact["robot_contact"],
            touches_peer=contact["utensil_contact"],
        )
    return result


def markers(data, name):
    if name == "whisk":
        return np.vstack(
            (
                world_points(data, name, [(-0.148, 0, 0)]),
                data.site("whisk_0_0_tip").xpos,
                data.body(name).xpos,
            )
        )
    if name == "spatula":
        return np.vstack(
            (
                world_points(data, name, [(-0.213, 0, 0)]),
                world_points(data, "spatula_blade_2", [(0.025, 0, 0)]),
                data.body(name).xpos,
            )
        )
    return world_points(data, name, [(-0.077, 0, 0.006), (0.216, 0, 0.002), (0, 0, 0)])


def removal(model, saved, name, slots):
    """Bounded external test hand. All other bodies remain passive in this branch."""
    trial = restore(model, saved)
    hand = trial.body(name)
    first = hand.xipos.copy()
    rotation = hand.xmat.reshape(3, 3).copy()
    peers = {p: markers(trial, p) for p in NAMES if p != name}
    peer_motion = dict.fromkeys(peers, 0.0)
    force = trial.xfrc_applied[hand.id]
    mass = model.body_subtreemass[hand.id]
    velocity = np.zeros(6)
    owners = np.array([(model.geom(i).name or "").partition("_")[0] for i in range(model.ngeom)])
    is_lifted = owners == name
    is_obstacle = np.isin(owners, (*peers, "left", "right"))
    hit_obstacle = False
    period = model.opt.timestep
    for tick in range(1, round(0.80 / period) + 1):
        t = min(1, tick * period / 0.64)
        height = 0.100 * t * t * (3 - 2 * t)
        speed = 0.100 * 6 * t * (1 - t) / 0.64
        mujoco.mj_objectVelocity(model, trial, mujoco.mjtObj.mjOBJ_BODY, hand.id, velocity, 0)
        requested = (
            205 * (first + height * Z - hand.xipos) + 6.5 * (speed * Z - velocity[3:]) + mass * 9.81 * Z
        )
        force[:3] = requested / max(1, np.linalg.norm(requested) / 4.0)
        axes = hand.xmat.reshape(3, 3)
        angle = axes.T @ Rotation.from_matrix(rotation @ axes.T).as_rotvec()
        angular_speed = axes.T @ velocity[:3]
        # The polar moment about a slim handle is far smaller than its transverse moments.
        torque = axes @ (
            np.array([0.012, 0.50, 0.50]) * angle - np.array([0.00045, 0.018, 0.018]) * angular_speed
        )
        # The hand grips the handle. Compensate gravity about that grip.
        torque += np.cross(trial.subtree_com[hand.id] - hand.xipos, mass * 9.81 * Z)
        force[3:] = torque / max(1, np.linalg.norm(torque) / 0.14)
        mujoco.mj_step(model, trial)
        a, b = trial.contact.geom.T
        blocked = (is_lifted[a] & is_obstacle[b]) | (is_lifted[b] & is_obstacle[a])
        hit_obstacle |= bool(np.any(blocked & (trial.contact.dist <= 0)))
        for p, initial in peers.items():
            travel = np.linalg.norm(markers(trial, p) - initial, axis=1).max()
            peer_motion[p] = max(peer_motion[p], float(travel))
    end = slot_readings(model, trial)
    rise = hand.xipos[2] - first[2]
    clearance = extents(model, trial, name)[0][2] - RIM
    peer_seated = all(
        end[p]["slot"] == slots[p]["slot"] and end[p]["supported"] and end[p]["released"] for p in peers
    )
    good = (
        rise >= 0.090
        and clearance >= 0.010
        and max(peer_motion.values()) <= 0.005
        and peer_seated
        and not hit_obstacle
        and end[name]["released"]
    )
    return dict(
        start_sha256=state_id(saved),
        independent=bool(good),
        rise_mm=round(1000 * rise, 2),
        clearance_above_rim_mm=round(1000 * clearance, 2),
        peers_still_seated=bool(peer_seated),
        touched_robot_or_peer=hit_obstacle,
        max_peer_motion_mm={k: round(1000 * v, 3) for k, v in peer_motion.items()},
    )


def step_score(model, data):
    # Every handle origin lies inside its material. A tool outside the tray's
    # length rules out full credit, so only the threading test is needed there.
    if any(abs(data.body(n).xpos[0] - TRAY_X) > TRAY_LENGTH / 2 for n in NAMES):
        return 0.5 if any(not r["caught"] for r in engagement(data).values()) else 0.0
    return evaluate(model, data)["score"]


def evaluate(model, data):
    return evaluate_saved(model, snapshot(model, data))


def evaluate_saved(model, saved):
    trial = restore(model, saved)
    caught = engagement(trial)
    shape = whisk_shape(trial)
    slots = slot_readings(model, trial)
    full_candidate = (
        all(s["supported"] and s["released"] and not s["touches_peer"] for s in slots.values())
        and {s["slot"] for s in slots.values()} == {1, 2, 3}
        and shape["intact"]
        and not any(r["caught"] for r in caught.values())
    )
    # Rest must persist for a quarter second after the robot has released the tools.
    settled = False
    tests = {}
    if full_candidate:
        origins = {n: markers(trial, n) for n in NAMES}
        mujoco.mj_step(model, trial, nstep=round(0.25 / model.opt.timestep))
        settled = all(
            np.linalg.norm(markers(trial, n) - origins[n], axis=1).max() <= 0.003 for n in NAMES
        )
        if settled:
            tests = {n: removal(model, saved, n, slots) for n in NAMES}
    full = len(tests) == 3 and all(t["independent"] for t in tests.values())
    at_least_one_free = any(not r["caught"] for r in caught.values())
    score = 1.0 if full else 0.5 if at_least_one_free else 0.0
    return dict(
        score=score,
        state_sha256=state_id(saved),
        engagement=caught,
        whisk=shape,
        slots=slots,
        settled=bool(settled),
        removals=tests,
    )
