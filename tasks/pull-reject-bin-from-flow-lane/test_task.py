"""Offline checks of the flow-lane reject-bin scorer and demo, no simulator. The three bins and the two grippers are
moved along hand-made paths at 20 Hz and fed through success() then score(), as the simulator does. Covers every tier,
every failure in the scoring text and adversarial cases: doing nothing, the full job, a hand left on a lane bin, the
back bin running, caught partway, the 2 cm and 8 cm boundaries, lifting or pushing the back bin instead of holding it,
the tagged bin still held, tipped, hovering or beside the tote, a lane bin dropped (latched), lifted out or tipped and
put back (recovered) or left off the lane, the wrong bin in the tote, D frozen once the tagged bin is in, a new trial,
the thresholds in the text, and the demo's plan. Run without FLOW_TAG_SLOT, it runs every check once with the tag in
slot 2 and once in slot 3 (one process at a time), and checks that an unset slot is drawn at random.

    PYTHONPATH=<yam_arena stubs or the real package> python3 test_task.py
"""
import json
import math
import os
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent

if os.environ.get("FLOW_TAG_SLOT") not in ("2", "3"):
    rc = 0
    for k in (2, 3):
        print(f"== tag in slot {k}", flush=True)
        rc |= subprocess.call([sys.executable, __file__], env={**os.environ, "FLOW_TAG_SLOT": str(k)})
    os.environ.pop("FLOW_TAG_SLOT", None)
    seen = {load(HERE / "task.py").module.TAG_SLOT for _ in range(16)}
    ok = seen == {2, 3}
    print(("ok   " if ok else "FAIL ") + f"FLOW_TAG_SLOT unset: the tag's slot is drawn at random ({sorted(seen)})")
    print("all pass" if rc == 0 and ok else "FAILED")
    raise SystemExit(0 if rc == 0 and ok else 1)

M = load(HERE / "task.py").module
import yamkit as kit  # noqa: E402

FAILS = []
Q_LANE = np.array(M.Q_SLOPE)
DOWN_X = Rotation.from_matrix([[1, 0, 0], [0, -1, 0], [0, 0, -1]]).as_quat()  # pointing down, closing along x
DOWN_Y = Rotation.from_matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]]).as_quat()  # pointing down, closing along y
AWAY = {"left_gripper": (0.15, 0.28, M.TOP + 0.30), "right_gripper": (0.15, -0.28, M.TOP + 0.30)}
WALL_OPEN = M.WALL / (2 * 0.0475)  # fingers closed on a 3 mm wall


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        FAILS.append(what)


def bin_state(pos, quat, vel=(0, 0, 0)):
    p = np.asarray(pos, float)
    r = Rotation.from_quat(quat).as_matrix()
    c = np.array([[sx * M.BIN_W / 2, sy * M.BIN_L / 2, z] for sx in (-1, 1) for sy in (-1, 1) for z in (0, M.BIN_H)])
    w = p + c @ r.T
    return ObjectState(pos=p, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3),
                       low=w.min(0), high=w.max(0))


def lane_pose(s, u=0.0, h=0.0):
    return M.lane_point(u, s, h), Q_LANE


class Trial:
    """Holds the scene's poses; step() builds one control step's objects and scores it."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        kit._EPISODE["last"] = None
        self.bins = {n: list(lane_pose(M.S_SLOT[M.SLOT_OF[n]])) for n in M.BINS}
        self.vel = {n: (0, 0, 0) for n in M.BINS}
        self.grip = {g: (AWAY[g], DOWN_X, 1.0) for g in M.GRIPPERS}
        self.s = 0.0
        self.step()

    def step(self, n=1):
        for _ in range(n):
            o = {k: bin_state(p, q, self.vel[k]) for k, (p, q) in self.bins.items()}
            for g, (p, q, op) in self.grip.items():
                p = np.asarray(p, float)
                o[g] = ObjectState(pos=p, quat_xyzw=np.asarray(q, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                                   low=p - 0.013, high=p + 0.013)
                kit._OPENING[g] = op
            M.success(o)
            self.s = M.score(o)
        return self.s

    def put(self, name, pos, quat=None, steps=1):
        self.bins[name] = [np.asarray(pos, float), Q_LANE if quat is None else np.asarray(quat, float)]
        return self.step(steps)

    def slide_back(self, to_s, steps=8, lift=0.0, name="back_bin"):
        """Move a lane bin (the back bin by default) along the lane (and up off it by `lift`) to s = to_s."""
        s0 = M.to_lane(self.bins[name][0])[1]
        for s in np.linspace(s0, to_s, steps):
            self.bins[name][0] = M.lane_point(0.0, s, lift)
            self.step()

    def hold_back(self, on=True):
        self.grip["left_gripper"] = (M.back_grip(), DOWN_X, WALL_OPEN) if on else (AWAY["left_gripper"], DOWN_X, 1.0)

    def carry_tagged_to_tote(self, release=True, tilt=0.0, z_above=0.0, at=None):
        """Right hand pinches the tagged bin's near wall, lifts it, carries it over the tote and sets it down."""
        start = np.array(self.bins["tagged_bin"][0])
        goal = np.array([M.TOTE_C[0], M.TOTE_C[1], M.TOTE_FLOOR + z_above]) if at is None else np.asarray(at, float)
        q = Rotation.from_euler("x", tilt, degrees=True).as_quat()
        path = [start + np.array([0, 0, 0.09 * k / 6]) for k in range(1, 7)]
        top = path[-1]
        path += [top + (np.array([goal[0], goal[1], top[2]]) - top) * k / 6 for k in range(1, 7)]
        path += [np.array([goal[0], goal[1], top[2] + (goal[2] - top[2]) * k / 6]) for k in range(1, 7)]
        for p in path:
            self.bins["tagged_bin"] = [p, Q_LANE]
            self.grip["right_gripper"] = (p + np.array([-(M.BIN_W - M.WALL) / 2, 0, M.BIN_H - M.GRIP_BELOW_RIM]),
                                          DOWN_X, WALL_OPEN)
            self.step()
        self.bins["tagged_bin"] = [goal, q]
        self.step()
        if release:
            self.grip["right_gripper"] = (AWAY["right_gripper"], DOWN_X, 1.0)
        return self.step()

    def close_up(self):
        """The lane bins roll down into the gap, against the stop and each other, in lane order."""
        for i, n in enumerate(M.LANE_BINS):
            target = M.S_SLOT[i + 1] + 0.001 * i
            if abs(M.to_lane(self.bins[n][0])[1] - target) > 1e-6:
                self.slide_back(target, steps=6, name=n)


def full_job(t, d_before=0.0):
    t.hold_back()
    t.step(5)
    if d_before:
        t.slide_back(M.S_BACK - d_before, steps=4)
    t.carry_tagged_to_tote()
    t.step(5)
    t.hold_back(False)
    t.close_up()
    return t.step(25)


# -- tiers ------------------------------------------------------------------------------------------------------------
t = Trial()
check(t.step(60) == 0.0, "doing nothing scores 0")
o0 = {n: bin_state(*t.bins[n]) for n in M.BINS}
check(len(M.BINS) == 4 and not any(M.off_lane(o0, n) for n in M.BINS) and all(M.on_wheels(o0, n) for n in M.BINS),
      "start: all four bins sit on the wheels, in the lane")
check(M.BINS.index("tagged_bin") + 1 == M.TAG_SLOT and M.BINS[M.TAG_SLOT] == "back_bin",
      f"the tagged bin is in slot {M.TAG_SLOT} and the back bin is the next one up the lane")

t = Trial()
check(full_job(t) == 1.0, "full job: held, in the tote, let go, lane closed up: 1.0")
check(M.travel() < 1e-9, "D = 0 when held")

t = Trial()
t.hold_back(); t.step(5); t.carry_tagged_to_tote(); t.step(5)
check(t.step(25) == 0.75, "tagged bin in, but the left hand still holds the back bin: 0.75")
t.hold_back(False); t.step(3)
check(t.step(25) == 0.75, "let go but the back bin has not rolled down (left where it was): 0.75")
t.close_up()
check(t.step(25) == 1.0, "then it rolls down: 1.0")

t = Trial()
t.hold_back(); t.step(5); t.carry_tagged_to_tote()
t.close_up()
t.grip["left_gripper"] = (np.array(t.bins["back_bin"][0]) + (M.back_grip() - M.lane_point(0.0, M.S_BACK, 0.0)),
                          DOWN_X, WALL_OPEN)
check(t.step(25) == 0.75, "closed up but a hand still on the back bin: 0.75")

t = Trial()  # no hold: the back bin runs down the 13.7 cm gap while the tagged bin is still over the lane
start = np.array(t.bins["tagged_bin"][0])
t.bins["tagged_bin"][0] = start + np.array([0, 0, 0.09]); t.step()
t.close_up()
t.carry_tagged_to_tote(); t.step(25)
check(t.s == 0.25, "back bin ran (D 13.7 cm) before the tagged bin was in: 0.25")

for d, want in ((0.019, 1.0), (0.021, 0.5), (0.05, 0.5), (0.079, 0.5), (0.081, 0.25)):
    t = Trial()
    check(full_job(t, d_before=d) == want, f"back bin moved {d * 100:.1f} cm before: {want}")

t = Trial()  # lifted the back bin 3 cm off the wheels to stop it rolling, then set it back
t.hold_back(); t.slide_back(M.S_BACK, steps=3, lift=0.03); t.carry_tagged_to_tote(); t.slide_back(M.S_BACK, 3, 0.0)
t.hold_back(False); t.close_up()
check(t.step(25) == 0.5, "back bin lifted 3 cm off the lane instead of held: D 3 cm, 0.5")

t = Trial()  # pushed the back bin 3 cm up the lane first
t.hold_back(); t.slide_back(M.S_BACK + 0.03, steps=4); t.carry_tagged_to_tote(); t.hold_back(False); t.close_up()
check(t.step(25) == 0.5, "back bin pushed 3 cm up the lane: D 3 cm, 0.5")

# -- the tagged bin -----------------------------------------------------------------------------------------------------
t = Trial()
t.hold_back(); t.step(3); t.carry_tagged_to_tote(release=False); t.hold_back(False); t.close_up()
check(t.step(40) == 0.0, "tagged bin in the tote but still held: 0")
t.grip["right_gripper"] = (AWAY["right_gripper"], DOWN_X, 1.0)
check(t.step(5) == 0.0, "let go, under 1 s: 0")
check(t.step(20) == 1.0, "let go and still for 1 s: 1.0")

t = Trial()
t.hold_back(); t.carry_tagged_to_tote(tilt=30); t.hold_back(False); t.close_up()
check(t.step(25) == 0.0, "tagged bin in the tote tipped 30 degrees: 0")
t = Trial()
t.hold_back(); t.carry_tagged_to_tote(tilt=15); t.hold_back(False); t.close_up()
check(t.step(25) == 1.0, "tagged bin in the tote tilted 15 degrees (on the bolts' bag): 1.0")

t = Trial()
t.hold_back(); t.carry_tagged_to_tote(z_above=0.05); t.hold_back(False); t.close_up()
check(t.step(25) == 0.0, "tagged bin 5 cm above the tote's floor (resting on the rim or held up): 0")

t = Trial()
t.hold_back(); t.carry_tagged_to_tote(at=(0.42, 0.20, M.TOP)); t.hold_back(False); t.close_up()
check(t.step(25) == 0.0, "tagged bin set on the table beside the tote: 0")

t = Trial()
t.hold_back(); t.carry_tagged_to_tote(); t.hold_back(False); t.close_up()
t.vel["tagged_bin"] = (0.05, 0, 0)
check(t.step(25) == 0.0, "tagged bin still sliding in the tote: 0")

# -- a dropped bin (latched) and recoverable mistakes (judged at the end) -----------------------------------------------
t = Trial()
p0 = t.bins["front_bin"][0].copy()
t.put("front_bin", p0 + np.array([0.12, 0, -0.02]), steps=2)
t.put("front_bin", p0)
check(full_job(t) == 1.0, "front bin lifted off the lane (held, not falling) then put back: recovered, 1.0")

t = Trial()
p0 = t.bins["other_bin"][0].copy()
t.vel["other_bin"] = (0, 0, -0.8)
t.put("other_bin", p0 + np.array([0.12, 0, -0.03]), steps=1)
t.vel["other_bin"] = (0, 0, 0)
t.put("other_bin", p0)
check(full_job(t) == 0.0, "a lane bin falling outside the lane (0.8 m/s down), then put back: dropped, latched 0")

t = Trial()
t.vel["front_bin"] = (0, 0, -0.8)
t.step(2)
t.vel["front_bin"] = (0, 0, 0)
check(full_job(t) == 1.0, "moving down fast inside the lane is not a drop: 1.0")

t = Trial()
t.put("back_bin", t.bins["back_bin"][0], Rotation.from_euler("y", 40, degrees=True).as_quat(), steps=3)
t.put("back_bin", lane_pose(M.S_BACK)[0], Q_LANE)
check(full_job(t) == 1.0, "back bin tipped 40 degrees in the lane, then stood back: recovered, 1.0")

t = Trial()
full_job(t)
t.put("front_bin", t.bins["front_bin"][0], Rotation.from_euler("y", 40, degrees=True).as_quat(), steps=25)
check(t.s == 0.0, "a lane bin left tipped 40 degrees at the end: 0")

t = Trial()
full_job(t)
t.put("other_bin", t.bins["other_bin"][0] + np.array([0.15, 0, -0.02]), steps=25)
check(t.s == 0.0, "a lane bin left off the lane at the end: 0")

t = Trial()  # the wrong bin: the back bin carried to the tote, the tagged bin left in the lane
t.put("back_bin", np.array([M.TOTE_C[0], M.TOTE_C[1], M.TOTE_FLOOR]), np.array([0, 0, 0, 1.0]), steps=30)
check(t.s == 0.0, "back bin put in the tote instead: 0")

t = Trial()  # D is frozen once the tagged bin is in the tote: the back bin rolling down after that is the lane's job
t.hold_back(); t.carry_tagged_to_tote(); t.hold_back(False); t.close_up()
check(t.step(25) == 1.0 and M.travel() < 0.001, "back bin rolls 13.7 cm after the tagged bin is in: D stays 0")

t = Trial()
check(t.step(5) == 0.0, "a new trial starts afresh")

# -- the text ---------------------------------------------------------------------------------------------------------
tj = json.loads((HERE / "task.json").read_text())
readme = (HERE / "README.md").read_text()
check(tj["instruction"] == M.INSTRUCTION, "task.json instruction == task.py INSTRUCTION")
for field in ("scoring",):
    txt = tj[field]
    for needle in ("2 cm", "8 cm", "1 cm", "1.5 cm", "20 degrees", "30 degrees", "10 degrees", "1 second"):
        check(needle in txt, f"scoring text has {needle!r}")
    for needle in ("0.25", "0.5", "0.75", "1.0"):
        check(needle in txt, f"scoring text lists {needle}")
for needle in ("13.7 x 10.5 x 7.6 cm", "0.35 kg", "3 degrees", "12 cm", "18 x 26 x 10 cm", "0.03"):
    check(needle in tj["setup"] + tj["fidelity"], f"setup/fidelity has {needle!r}")
    check(needle in readme, f"README has {needle!r}")
check(math.isclose(M.HELD_D, 0.02) and math.isclose(M.CAUGHT_D, 0.08) and math.isclose(M.CLOSED_GAP, 0.01)
      and math.isclose(M.IN_TOTE_Z, 0.015) and M.TOTE_TILT == 20 and M.OFF_TILT == 30 and M.LANE_TILT == 10,
      "code thresholds match the text")
check(re.search(r"2,400 steps", tj["run_limit"]) is not None and M.RUN_LIMIT_STEPS == 2400, "run limit 2,400 steps")

# -- the demo's plan ----------------------------------------------------------------------------------------------------
plan = M.demo()
for side, wps in plan.items():
    for w in wps:
        x, y, z = w.pos
        check(0.15 <= x <= 0.40 and abs(y) <= 0.30 and z <= M.TOP + 0.22, f"{side} '{w.label}' in reach {np.round(w.pos, 3)}")
        check(all(type(v) is float for v in w.pos), f"{side} '{w.label}' position is plain Python floats")


def at(wps, t_):
    t0 = 0.0
    for w in wps:
        if t_ <= t0 + w.duration:
            return np.asarray(w.pos)
        t0 += w.duration
    return np.asarray(wps[-1].pos)


T = max(sum(w.duration for w in v) for v in plan.values())
gap = min(abs(at(plan["left"], s)[1] - at(plan["right"], s)[1]) for s in np.arange(0, T, 0.1))
check(gap >= 0.15, f"arms at least 15 cm apart in y at every waypoint end ({gap:.3f} m)")
rl = sum(w.duration for w in plan["right"][:8])
ll = sum(w.duration for w in plan["left"][:4])
check(ll >= rl + 0.9, "left hand lets go only after the right has let go of the tagged bin")
check(sum(w.duration for w in plan["left"][:3]) <= sum(w.duration for w in plan["right"][:3]),
      "left hand holds the queue before the right hand touches the tagged bin")
check(T / M.DT <= 600, f"demo fits 600 steps ({T / M.DT:.0f})")

# the planned grips really put the pads on the walls (guardkit's contact check)
t = Trial()
t.grip["right_gripper"] = (M.tagged_grip(), DOWN_X, WALL_OPEN)
t.grip["left_gripper"] = (M.back_grip(), DOWN_X, WALL_OPEN)
o = {k: bin_state(p, q) for k, (p, q) in t.bins.items()}
for g, (p, q, op) in t.grip.items():
    o[g] = ObjectState(pos=np.asarray(p), quat_xyzw=np.asarray(q), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                     low=np.asarray(p), high=np.asarray(p))
    kit._OPENING[g] = op
check("right_gripper" in M.touched(o, "tagged_bin"), "right grip pose touches the tagged bin")
check("left_gripper" in M.touched(o, "back_bin"), "left grip pose touches the back bin")
check(all("right_gripper" not in M.touched(o, n) for n in M.BINS if n != "tagged_bin")
      and all("left_gripper" not in M.touched(o, n) for n in M.BINS if n != "back_bin"),
      "grips touch only their own bin")

# the lane keeps its own low-friction material: yamkit's scene hook (through flowkit) spawns it with no spawner
# material, which Isaac Lab would bind over it; the bins and the tote keep the grippy one
import types  # noqa: E402

import flowkit as fk  # noqa: E402

GRIPPY = object()
fake = types.ModuleType("fake_scene")
fake.__dict__.update(
    _grippy=lambda: GRIPPY, _contact_props=lambda: None, _spawn_fixture=None, task_object=lambda spec: spec,
    object_states=lambda env, names: {}, PhysxRigidBodyPropertiesCfg=lambda **kw: kw, Pose=lambda **kw: kw,
    ObjectType=types.SimpleNamespace(RIGID="rigid"), Object=lambda **kw: kw)
kit._patch(fake)
lane = fake.task_object(M.LANE)
check(lane["spawn_cfg_addon"]["physics_material"] is None, "the lane is spawned with no spawner material")
check(all(fake.task_object(p)["spawn_cfg_addon"]["physics_material"] is GRIPPY for p in (*M.LANE_PROPS, M.TOTE)),
      "bins and tote keep the grippy material")
check(fake._grippy() is GRIPPY, "the spawner's material function is restored")
usd = kit._usda(M.LANE)
check(f"physics:staticFriction = {M.MU_BED}" in usd and 'frictionCombineMode = "min"' in usd
      and usd.count("material:binding:physics = </Root/PhysMat>") == len(M.LANE.colliders),
      "the lane's USD binds its 0.03 'min' material to every collider")
check(isinstance(M.LANE, fk.FrictionProp), "the lane is a FrictionProp")

print("all pass" if not FAILS else f"{len(FAILS)} FAILED")
raise SystemExit(1 if FAILS else 0)
