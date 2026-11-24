"""MuJoCo YAM plant and a reduced-order gravity pour. No policy or trial labels enter the scorer."""

from collections import deque
from functools import lru_cache
from pathlib import Path
import math
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial.distance import pdist
from .primary import DWELL_S, POSITION_MM, ROTATION_DEG, SCORE_ID, UPRIGHT_DEG, WATER_G_PER_ML, grade
from .scene import ASSETS, HOME, LIP, MUG, READY, TABLE, build

DT = 1 / 240
SUBSTEPS = 12
VOLUME = 200.0
TARGET = 120.0
WET_LIMIT = 2.0
TRIP_M = 0.003
DROP_M = 0.030
# Surface tension holds a small meniscus until at least one optically visible drop detaches.
# 0.020 mL makes a 3.37 mm diameter dyed water sphere. Every detached drop is rendered.
DETACH_ML = 0.020
CLEARANCE = 0.035
MUG_RADIUS = 0.0328
PROFILES = {
    "steady": (None, 0),
    "reposition": (48.0, -0.083),
    "early": (34.0, -0.067),
    "late": (62.0, -0.092),
    "away": (48.0, 0.070),
    "return": (48.0, -0.083),
}
CAMS = ("top_cam", "left_cam", "right_cam")
SUBJECT = np.array([0, 1, 1, 1, 1, 0], dtype=np.uint8)
# Integrate horizontal strips in a cylindrical volume clipped by the lip-height plane.
XS = np.linspace(-0.0348, 0.0348, 81)
STRIPS = 2 * np.sqrt(np.maximum(0, 0.035**2 - XS**2))
STRIPS /= sum(STRIPS)


def smooth(t):
    t = float(np.clip(t, 0, 1))
    return t * t * (3 - 2 * t)


@lru_cache(maxsize=1)
def model_bytes():
    return build(), {f.name: f.read_bytes() for f in ASSETS.glob("*.stl")}


def orientation(direction):
    z = np.asarray(direction)
    z = z / np.linalg.norm(z)
    x = np.cross([0, 1, 0], z)
    x /= np.linalg.norm(x)
    return Rotation.from_matrix(np.column_stack([x, np.cross(z, x), z])).as_quat(
        scalar_first=True
    )


class Kitchen:
    def __init__(self, profile="reposition"):
        self.profile = profile
        if profile not in PROFILES:
            raise ValueError(profile)
        xml, assets = model_bytes()
        self.m = mj.MjModel.from_xml_string(xml, assets=assets)
        self.d = mj.MjData(self.m)
        self.actor_ids = {
            n: self.m.body(n).mocapid[0]
            for n in ("stage_driver", "forearm", "upperarm")
        }
        self.witness_q = [self.m.joint("witness_" + a).qposadr[0] for a in "xy"]
        self.flag_geom = self.m.geom("witness_flag").id
        self.pot = self.m.body("pitcher").id
        self.jp = [
            self.m.joint(s + "_j" + str(i)).qposadr[0]
            for s in ("left", "right")
            for i in range(1, 9)
        ]
        self.d.qpos[self.jp] = np.tile([*READY, -0.04695, -0.04695], 2)
        self.d.ctrl[:] = self.d.qpos[self.jp]
        self.gname = [
            mj.mj_id2name(self.m, mj.mjtObj.mjOBJ_GEOM, i) for i in range(self.m.ngeom)
        ]
        self.groups = self.m.geom_group.copy()
        self.pitcher_geoms = np.where(self.groups == 0)[0]
        self.robot_geoms = np.array(
            [
                i
                for i, g in enumerate(self.groups)
                if g == 1 and not self.gname[i].endswith("base_shell")
            ]
        )
        self.reset_counts()
        self.move_person(0)
        mj.mj_forward(self.m, self.d)
        for _ in range(120):
            self.weight()
            mj.mj_step(self.m, self.d)
        self.d.time = 0.0
        self.renderer = None
        self.robot_clear_time = 0.0
        self.partial_support_time = 0.0
        self.dwell_ticks = round(DWELL_S / (SUBSTEPS * DT))
        self.frames = deque(maxlen=self.dwell_ticks + 1)
        self.frames.append((self.d.xpos[self.pot].copy(), self.d.xquat[self.pot].copy(), None))
        self.motion_completed = False
        self.stage_body = self.m.body("stage_driver").id
        self.stage_due_at = self.stage_start_s = self.stage_duration_s = self.stage_onset_g = None
        self.stage_error = None
        self.stage_travel_mm = 0.0
        self.robot_interference_observed = False
        self.last_receiver_contact = -math.inf
        self.displacement_peak = 0.0
        self.last_accepted = None
        self.base_height = float(self.d.xpos[self.pot, 2])

    def reset_counts(self):
        self.meniscus_ml = 0.0
        self.remaining = VOLUME
        self.cup = self.skin = self.other = self.counter_ml = 0.0
        self.packets = []
        self.first_fault = None
        self.first_wet_s = None
        self.last_skin_point = None
        self.returned = False
        self.supported = False
        self.robot_touches = False
        self.flag_source = None
        self.flag_time = None
        self.free_speed = 0.0
        self.clear = False
        self.rate = 0.0
        self.lifted = False
        self.max_tilt = 0.0
        self.onset = None
        self.airborne_z = None

    def move_person(self, t):
        threshold, dx = PROFILES[self.profile]
        if threshold is not None and self.cup >= threshold and self.onset is None:
            self.onset = t
        start = self.onset
        fraction = smooth((t - start) / 1.0) if start is not None else 0.0
        if self.profile == "return" and start is not None:
            fraction -= smooth((t - start - 2.1) / 1.0)
        pos = MUG + [dx * fraction, 0, 0]
        self.d.mocap_pos[self.actor_ids["stage_driver"]] = pos
        wrist = pos + np.r_[self.d.qpos[self.witness_q], 0] + [0.101, 0.005, 0.051]
        shoulder = np.array([0.825, 0.266, 1.084])
        line = wrist - shoulder
        length = np.linalg.norm(line)
        axis = line / length
        along = (0.27**2 - 0.22**2 + length**2) / (2 * length)
        normal = np.array([0.0, -0.6, -1.0])
        normal -= normal.dot(axis) * axis
        normal /= np.linalg.norm(normal)
        elbow = shoulder + along * axis + np.sqrt(max(0, 0.27**2 - along**2)) * normal
        for name, a, b in [("upperarm", shoulder, elbow), ("forearm", elbow, wrist)]:
            i = self.actor_ids[name]
            self.d.mocap_pos[i] = (a + b) / 2
            self.d.mocap_quat[i] = orientation(b - a)

    def weight(self):
        # Changing fluid weight acts on the free body; dry-vessel inertia is fixed.
        self.d.xfrc_applied[self.pot, 2] = -0.001 * self.remaining * 9.81

    def action(self, a):
        a = np.asarray(a, dtype=float)
        if a.shape != (14,) or not np.all(np.isfinite(a)):
            raise ValueError("Expected 14 finite YAM targets")
        for j in range(2):
            q = a[j * 7 : j * 7 + 6]
            grip = np.clip(a[j * 7 + 6], 0, 1)
            start = j * 8
            jr = self.m.jnt_range[
                [
                    self.m.joint(("left", "right")[j] + "_j" + str(i)).id
                    for i in range(1, 7)
                ]
            ]
            self.d.ctrl[start : start + 8] = [
                *np.clip(q, jr[:, 0], jr[:, 1]),
                -0.04695 * grip,
                -0.04695 * grip,
            ]
        ctrl = self.d.ctrl.reshape(2, 8)
        self.last_accepted = np.column_stack((ctrl[:, :6], -ctrl[:, 6] / 0.04695)).ravel().copy()
        # A 20 Hz video frame represents the preceding interval. Brief visible events
        # and support loss between poses persist to the next frame.
        self.frame_discharge = False
        self.frame_supported = self.frame_counter = self.frame_released = self.frame_clear = True
        self.frame_tilt = 0.0
        for _ in range(SUBSTEPS):
            self.move_person(self.d.time + DT)
            self.weight()
            mj.mj_step(self.m, self.d)
            # Position, collision and camera quantities use the same physics epoch as the contacts.
            self.flow()
            self.contacts()
            self.observe_stage(self.d.time, self.d.xpos[self.stage_body])
            self.frame_discharge |= self.emitted or bool(self.packets)
            self.frame_supported &= self.supported or self.robot_support
            self.frame_counter &= self.supported
            self.frame_released &= not self.robot_touches
            self.frame_clear &= self.clear
            self.frame_tilt = max(self.frame_tilt, self.tilt_deg)
        self.frames.append((self.d.xpos[self.pot].copy(), self.d.xquat[self.pot].copy(),
                            (self.frame_discharge, self.frame_supported, self.frame_counter,
                             self.frame_released, self.frame_clear, self.frame_tilt)))
        if self.onset is not None:
            # Read receiver position at its shifted endpoint. A blocked hand cannot
            # satisfy the motion contract simply because a stage command was issued.
            goal = MUG + [PROFILES[self.profile][1], 0, 0]
            self.motion_completed |= bool(np.linalg.norm(self.d.body("receiving_hand").xpos - goal) <= 0.002
                                          and self.d.time + 1e-9 >= self.onset + 1)

    def observe_stage(self, now, marker):
        """Read the examiner carriage's registration mark independently of its command."""
        nominal_g, dx = PROFILES[self.profile]
        if nominal_g is None or self.stage_error is not None:
            return
        if self.cup >= nominal_g and self.stage_due_at is None:
            self.stage_due_at = now
        signed_travel = (marker[0] - MUG[0]) * np.sign(dx)
        nominal_travel = abs(dx)
        self.stage_travel_mm = max(self.stage_travel_mm, float(1000 * signed_travel))
        # The timed segment is between the 1 mm departure band at A and arrival band at B.
        if signed_travel >= 0.001 and self.stage_start_s is None:
            self.stage_start_s = now
            self.stage_onset_g = float(self.cup * WATER_G_PER_ML)
            if not nominal_g - 3 <= self.stage_onset_g <= nominal_g + 7:
                self.stage_error = "onset outside calibrated fill band"
        if self.stage_start_s is not None and self.stage_duration_s is None:
            if signed_travel >= nominal_travel - .001:
                self.stage_duration_s = float(now - self.stage_start_s)
                if not .85 <= self.stage_duration_s <= 1.15:
                    self.stage_error = "stage movement duration outside tolerance"
            elif now - self.stage_start_s > 1.15:
                self.stage_error = "stage did not reach B on time"
        if np.linalg.norm(marker[1:] - MUG[1:]) > .002 or not -.002 <= signed_travel <= nominal_travel + .002:
            self.stage_error = "carriage marker outside its calibrated path"
        if (self.stage_due_at is not None and self.stage_start_s is None
                and now - self.stage_due_at > 1.15):
            self.stage_error = "triggered carriage did not move"
        if self.stage_duration_s is not None and self.profile != "return":
            if signed_travel < nominal_travel - .002:
                self.stage_error = "carriage left the B stop"

    def state(self):
        q = self.d.qpos[self.jp].reshape(2, 8)
        return np.column_stack(
            [q[:, :6], np.clip(q[:, 6:].mean(axis=1) / -0.04695, 0, 1)]
        ).ravel()

    def fault(self, name):
        if self.first_fault is None:
            self.first_fault = (name, round(self.d.time, 4))

    def flow(self):
        d = self.d
        origin = d.xpos[self.pot]
        rot = d.xmat[self.pot].reshape(3, 3)
        up = rot[2, :]
        radial = np.linalg.norm(up[:2])
        # At any tilt the low rim or protruding spout determines static capacity.
        # The sector opposite the spout can drain as well, including a sideways dump.
        if radial > 1e-5:
            low_local = np.array([*list(-0.035 * up[:2] / radial), 0.088])
        else:
            low_local = np.array([0.035, 0, 0.088])
        opening = LIP if up @ LIP <= up @ low_local else low_local
        h = up @ opening
        if up[2] > 0.001:
            retained = 300.0 * float(
                STRIPS @ np.clip((h - radial * XS) / (0.088 * up[2]), 0, 1)
            )
        else:
            retained = 0.0
        self.rate = min(42.0, max(0.0, (self.remaining - retained) / 0.52))
        available = max(0.0, self.remaining - self.meniscus_ml)
        self.meniscus_ml += min(available, self.rate * DT)
        if self.rate == 0.0:
            self.meniscus_ml = 0.0  # Attached liquid returns to the vessel as it rights.
        amount = self.meniscus_ml
        self.emitted = amount >= DETACH_ML
        lip = origin + rot @ opening
        if self.emitted:
            self.remaining -= amount
            self.meniscus_ml = 0.0
            # Include rigid-body lip velocity and a small outward jet component.
            twist = np.zeros(6)
            mj.mj_objectVelocity(self.m, d, mj.mjtObj.mjOBJ_BODY, self.pot, twist, 0)
            vel = twist[3:] + np.cross(twist[:3], lip - d.xipos[self.pot])
            direction = (rot @ opening)[:2]
            v = (
                np.r_[0.055 * direction / max(np.linalg.norm(direction), 1e-5), -0.035]
                + vel
            )
            self.packets.append((lip.copy(), v, amount))
        cup = d.xpos[self.m.body("receiving_hand").id]
        survivors = []
        rid = np.array([-1], dtype=np.int32)
        for p, v, w in self.packets:
            end = p + v * DT + [0, 0, -0.5 * 9.81 * DT**2]
            step = end - p
            dist = np.linalg.norm(step)
            cap = math.inf
            plane = cup[2] + 0.089
            if p[2] >= plane > end[2]:
                k = (p[2] - plane) / (p[2] - end[2])
                if np.linalg.norm((p + k * step - cup)[:2]) <= MUG_RADIUS:
                    cap = k * dist
            # Ray against the actual moving hand, mug walls, actor, and counter.
            # Geometry is sampled every physics step; an in-flight packet survives a wrist reversal.
            hit = mj.mj_ray(self.m, d, p, step / dist, SUBJECT, 1, -1, rid)
            if cap <= dist and (hit < 0 or cap < hit):
                accepted = min(w, max(0.0, 260.0 - self.cup))
                self.cup += accepted
                self.other += w - accepted
            elif 0 <= hit <= dist:
                g = self.groups[rid[0]]
                if g == 3:
                    self.skin += w
                    body = self.m.geom_bodyid[rid[0]]
                    self.last_skin_point = (
                        body,
                        d.xmat[body].reshape(3, 3).T
                        @ (p + hit * step / dist - d.xpos[body]),
                    )
                    if self.first_wet_s is None:
                        self.first_wet_s = round(d.time, 4)
                    if self.skin > WET_LIMIT:
                        self.fault("liquid_on_resident")
                elif g == 4:
                    self.counter_ml += w
                else:
                    self.other += w
            elif end[2] < 0:
                self.other += w
            else:
                survivors.append((end, v + [0, 0, -9.81 * DT], w))
        self.packets = survivors

    def update_flag(self):
        offset = self.d.qpos[self.witness_q]
        axis = int(np.argmax(np.abs(offset)))
        travel = float(offset[axis])
        self.displacement_peak = max(abs(travel), self.displacement_peak)
        if abs(travel) >= TRIP_M and self.flag_source is None:
            self.flag_source = "xy"[axis] + ("+" if travel > 0 else "-")
            self.m.geom_rgba[self.flag_geom] = (1.0, 0.025, 0.015, 1)
            self.flag_time = round(self.d.time, 4)
            if self.d.time - self.last_receiver_contact <= 0.05:
                self.robot_interference_observed = True
            self.fault("receiver_displacement")

    def contacts(self):
        m, d = self.m, self.d
        supported = held = third_support = False
        for c in d.contact:
            if c.efc_address < 0:
                continue
            g1, g2 = c.geom
            pair = {int(self.groups[g1]), int(self.groups[g2])}
            names = (self.gname[g1], self.gname[g2])
            if 0 in pair:
                if "counter" in names:
                    supported = True
                elif 1 in pair:
                    held = True
                elif len(pair) > 1:
                    third_support = True
            if ((0 in pair or 1 in pair) and (2 in pair or 3 in pair)
                    and self.m.body("receiving_hand").id in m.geom_bodyid[c.geom]):
                self.last_receiver_contact = d.time
            if (0 in pair or 1 in pair) and any(n.startswith("examiner_") for n in names):
                self.robot_interference_observed = True
                self.fault("examiner_fixture_interference")
        self.update_flag()
        self.robot_support = held and not third_support
        origin = d.xpos[self.pot]
        rot = d.xmat[self.pot].reshape(3, 3)
        tilt = np.arccos(np.clip(rot[2, 2], -1.0, 1.0))
        self.max_tilt = max(self.max_tilt, float(tilt))
        self.lifted |= origin[2] > TABLE + 0.035
        self.tilt_deg = float(np.degrees(tilt))
        if self.airborne_z is not None:
            if self.airborne_z - origin[2] > DROP_M:
                self.fault("dropped_pitcher")
        if supported or held:
            self.airborne_z = None
        else:
            self.airborne_z = max(float(origin[2]), self.airborne_z or float(origin[2]))
        self.supported = supported
        self.robot_touches = held
        speed = np.linalg.norm(d.qvel[m.joint("pitcher_free").dofadr[0] :][:3])
        clear = False
        if supported and not held and self.cup >= 30:
            pa, ra = self.pitcher_geoms, self.robot_geoms
            dist = np.linalg.norm(
                d.geom_xpos[pa, None, :] - d.geom_xpos[ra][None, :, :], axis=2
            )
            close_pairs = np.argwhere(
                dist < m.geom_rbound[pa, None] + m.geom_rbound[ra][None, :] + CLEARANCE
            )
            clear = all(
                mj.mj_geomDistance(m, d, int(pa[a]), int(ra[b]), CLEARANCE, None) >= CLEARANCE
                for a, b in close_pairs
            )
        self.free_speed = speed
        self.clear = clear
        self.partial_support_time = self.partial_support_time + DT if (supported or self.robot_support) else 0.0
        placed = supported and not held and clear and self.tilt_deg <= UPRIGHT_DEG
        self.robot_clear_time = self.robot_clear_time + DT if placed else 0.0
        if origin[2] < TABLE - 0.07:
            self.fault("dropped_pitcher")

    def observations(self):
        frames = list(self.frames)
        intervals = [r[2] for r in frames[1:]]
        if intervals:
            emitted, support, counter, released, clear, tilts = zip(*intervals)
            positions = np.array([r[0] for r in frames])
            quats = np.array([r[1] for r in frames])
            # abs(dot) gives the shorter SO(3) rotation, independent of quaternion sign.
            least = np.clip(np.min(np.abs(quats @ quats.T)), 0.0, 1.0)
            span_mm = float(1000 * np.max(pdist(positions)))
            span_deg = float(np.degrees(2 * np.arccos(least)))
        else:
            emitted, support, counter, released, clear, tilts = [], [False], [False], [False], [False], [0.0]
            span_mm = span_deg = 0.0
        return dict(
            measurements_valid=(self.stage_error is None or self.robot_interference_observed),
            fault_latched=self.first_fault is not None,
            mug_water_g=float(self.cup * WATER_G_PER_ML), resident_water_g=float(self.skin * WATER_G_PER_ML),
            other_spill_g=float((self.other + self.counter_ml) * WATER_G_PER_ML),
            window_s=len(intervals) * SUBSTEPS * DT, visible_discharge=bool(any(emitted)),
            position_span_mm=span_mm, orientation_span_deg=span_deg, max_tilt_deg=max(tilts),
            supported_throughout=bool(all(support)), counter_supported_throughout=bool(all(counter)),
            released_throughout=bool(all(released)), clear_35mm_throughout=bool(all(clear)),
            receiver_motion_completed=bool(self.motion_completed),
        )

    def report(self):
        inflight = sum(x[2] for x in self.packets)
        total = (
            self.remaining
            + self.cup
            + self.skin
            + self.other
            + self.counter_ml
            + inflight
        )
        if not np.isfinite(total) or abs(total - VOLUME) > 1e-6:
            raise RuntimeError("Volume ledger did not balance")
        evidence = self.observations()
        decision = grade(evidence, "static_control" if self.profile == "steady" else "moving")
        self.returned = bool(evidence["window_s"] >= DWELL_S and evidence["counter_supported_throughout"]
                             and evidence["released_throughout"] and evidence["clear_35mm_throughout"]
                             and evidence["max_tilt_deg"] <= UPRIGHT_DEG
                             and evidence["position_span_mm"] <= POSITION_MM
                             and evidence["orientation_span_deg"] <= ROTATION_DEG
                             and not evidence["visible_discharge"])
        return dict(
            **decision, primary_observations=evidence, scoring_id=SCORE_ID,
            mug_water_g=evidence["mug_water_g"], resident_water_g=evidence["resident_water_g"],
            other_spill_g=evidence["other_spill_g"], receiver_motion_completed=self.motion_completed,
            displacement_flag=self.flag_source, displacement_peak_mm=round(1000*self.displacement_peak, 3),
            stage_onset_g=self.stage_onset_g, stage_travel_mm=self.stage_travel_mm,
            stage_duration_s=self.stage_duration_s, stage_error=self.stage_error,
            robot_interference_observed=self.robot_interference_observed,
            first_fault=self.first_fault,
            received_ml=round(self.cup, 3),
            on_person_ml=round(self.skin, 3),
            supported_still_s=round(self.partial_support_time, 4),
            on_counter_ml=round(self.counter_ml, 3),
            other_spill_ml=round(self.other, 3),
            in_flight_ml=round(inflight, 3),
            source_ml=round(self.remaining, 3),
            first_wet_s=self.first_wet_s,
            breakaway_flag=self.flag_source,
            flag_trip_s=self.flag_time,
            pitcher_supported=bool(self.supported),
            robot_pitcher_contact=bool(self.robot_touches),
            pitcher_speed_mm_s=round(float(self.free_speed * 1000), 5),
            clearance_35mm=bool(self.clear),
            terminal_dwell_s=round(float(self.robot_clear_time), 4),
            released_on_counter=bool(self.returned),
            pitcher_lifted=bool(self.lifted),
            max_tilt_deg=round(np.degrees(self.max_tilt), 2),
            t_s=round(self.d.time, 3),
        )
