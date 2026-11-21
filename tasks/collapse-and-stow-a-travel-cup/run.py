"""Run joint-target scripts and record native contact-state data."""

import argparse
import json
import shutil
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from motion import Motion, HOME, SIDES, DOWN, FROM_LEFT, FROM_RIGHT, frame
from scene import build, CupPhysics, LOCATION, TABLE_TOP
from scoring import grade

RATE = 20
ACTION_LIMIT = 1600
DEFAULT_PACE = .80
SUBSTEPS = 25
OPEN = .0475
CLOSE = 0.0
SIDE_SLICE = {"left": slice(0,7), "right": slice(7,14)}
LID_GRIP = frame((0,1,-.20), (-1,0,0))
TAB_GRIP = frame((.72,0,-1), (0,1,0))
ARC_FRACTION = .62
TAB_BIAS = .0115
SLEEVE_GRIP = FROM_LEFT
SLEEVE_OFFSET = np.array((0.,-.009,-.007))
SLEEVE_CLOSE = .032
PULL_DIST = .006
CLOSURE_ARC_FRACTION = 1.0
CLOSURE_BIAS = np.array((.005,0.,.032))
PLUG_PRESS = np.array((-.007,0.,-.028))
CUP_BODIES = ("cup_base", "stage_1", "stage_2", "stage_3", "stopper", "heat_sleeve")


class Trial:
    def __init__(self, pace=DEFAULT_PACE):
        if not .4 <= pace <= 1.5:
            raise ValueError("Pace must lie between 0.4 and 1.5")
        self.pace = pace
        self.model = build()
        self.data = mujoco.MjData(self.model)
        self.physics = CupPhysics(self.model)
        self.motion = Motion(self.model)
        self.commands = {side: np.r_[HOME, OPEN] for side in SIDES}
        self.trace = []
        self.actions = []
        self.peak_efforts = np.zeros(self.model.nu)
        driven_joint_ids = self.model.actuator_trnid[:, 0]
        self.driven_dofs = self.model.jnt_dofadr[driven_joint_ids]
        self.peak_speeds = np.zeros(self.model.nu)
        bodies_by_geom = [self.model.body(body).name for body in self.model.geom_bodyid]
        self.is_cup_geom = np.isin(bodies_by_geom, CUP_BODIES)
        self.arm_by_geom = np.array([
            1 if name.startswith("left_") else 2 if name.startswith("right_") else 0
            for name in bodies_by_geom
        ])
        self.data.qpos[:16] = np.tile(np.r_[HOME, OPEN, -OPEN], 2)
        mujoco.mj_forward(self.model, self.data)
        self.states = [self.data.qpos.copy()]

    def _measure(self, phase, arms_touched, cup_touched):
        data = self.data
        names = ("cup_base", "heat_sleeve", "stage_3")
        positions = {name: np.round(data.body(name).xpos, 6).tolist() for name in names}
        axes = {name: np.round(data.body(name).xmat.reshape(3,3)[:,2], 6).tolist() for name in names}
        robots = {side: np.round(self.motion.feature(data, side)[0], 6).tolist() for side in SIDES}
        touching = {side: {} for side in SIDES}
        finger_names = {side: {"left": (f"{side}_lf_down",), "right": (f"{side}_rf_down",)} for side in SIDES}
        for index in range(data.ncon):
            contact = data.contact[index]
            geom_ids = (contact.geom1, contact.geom2)
            body_names = [self.model.body(self.model.geom_bodyid[geom_id]).name for geom_id in geom_ids]
            for side in SIDES:
                for jaw, markers in finger_names[side].items():
                    for pair, name in enumerate(body_names):
                        if name in markers and body_names[1-pair] in ("stopper", "heat_sleeve", "cup_base", "stage_1", "stage_2", "stage_3"):
                            key = f"{jaw}:{body_names[1-pair]}"
                            force = np.zeros(6)
                            mujoco.mj_contactForce(self.model, data, index, force)
                            touching[side][key] = touching[side].get(key, 0.)+float(force[0])
        self.trace.append(dict(phase=phase, time=round(data.time, 4), positions=positions, axes=axes,
                               hand_features=robots, contact_n=touching,
                               arms_touched=arms_touched, cup_touched=cup_touched,
                               stopper_deg=float(np.degrees(self.physics.tab_angle(data))),
                               sleeve_fit_n=self.physics.fit_force,
                               folds_mm=np.round(self.physics.folds(data)*1000., 4).tolist()))
        self.states.append(data.qpos.copy())

    def _tick(self, phase):
        if len(self.actions) >= ACTION_LIMIT:
            raise ValueError("Trial exceeded the 1600-action limit")
        self.data.ctrl[:] = np.concatenate([self.commands[side] for side in SIDES])
        self.actions.append(self.data.ctrl.copy())
        arms_touched = cup_touched = False
        for _ in range(SUBSTEPS):
            self.physics.apply(self.data, self.model.opt.timestep)
            mujoco.mj_step(self.model, self.data)
            np.maximum(self.peak_efforts, np.abs(self.data.actuator_force), out=self.peak_efforts)
            np.maximum(self.peak_speeds, np.abs(self.data.qvel[self.driven_dofs]), out=self.peak_speeds)
            for geom1, geom2 in self.data.contact.geom:
                if geom1 < 0 or geom2 < 0:
                    continue
                arm1, arm2 = self.arm_by_geom[[geom1, geom2]]
                arms_touched |= arm1 != 0 and arm2 != 0 and arm1 != arm2
                cup_touched |= ((arm1 != 0 and self.is_cup_geom[geom2])
                                or (arm2 != 0 and self.is_cup_geom[geom1]))
        self._measure(phase, bool(arms_touched), bool(cup_touched))

    def effort_summary(self):
        return {
            self.model.actuator(index).name: {
                "peak": float(peak), "cap": float(np.max(np.abs(self.model.actuator_forcerange[index]))),
                "units": "N" if index in (6, 13) else "N*m",
                "peak_speed": float(self.peak_speeds[index]),
                "speed_units": "m/s" if index in (6, 13) else "rad/s",
            }
            for index, peak in enumerate(self.peak_efforts)
        }

    def replay(self, commands):
        actions = np.load(commands, allow_pickle=False)["actions"]
        if actions.ndim != 2 or actions.shape[1] != 14 or not np.isfinite(actions).all():
            raise ValueError("Expected finite 14-column joint target sequence")
        for action in actions:
            self.commands = {side: action[SIDE_SLICE[side]].copy() for side in SIDES}
            self._tick("command playback")

    def hold(self, phase, duration):
        for _ in range(round(RATE*duration)):
            self._tick(phase)

    def joints(self, phase, duration, **target):
        start = {side: self.commands[side].copy() for side in target}
        for amount in np.linspace(0,1,round(self.pace*duration*RATE)+1)[1:]:
            ramp = amount**2*(3.-2.*amount)
            for side, value in target.items():
                self.commands[side] = start[side]+ramp*(value-start[side])
            self._tick(phase)

    def at(self, phase, duration, **target):
        joint_target = {side: np.r_[self.motion.solve(side, pos, orientation, self.commands[side][:6]), jaw]
                        for side, (pos, orientation, jaw) in target.items()}
        self.joints(phase, duration, **joint_target)

    def path(self, phase, duration, side, target, rotation, grip, angular_tolerance=.052):
        initial, rotation_start = self.motion.feature(self.data, side)
        slerp = Slerp((0.,1.),Rotation.from_matrix((rotation_start,rotation)))
        for amount in np.linspace(0,1,round(self.pace*duration*RATE)+1)[1:]:
            ramp = amount**2*(3.-2.*amount)
            goal = initial + ramp*(target-initial)
            approach = slerp(ramp).as_matrix()
            self.commands[side] = np.r_[self.motion.solve(side, goal, approach, self.commands[side][:6], angular_tolerance), grip]
            self._tick(phase)

    def jaw(self, phase, side, grip, duration=.5):
        self.joints(phase, duration, **{side: np.r_[self.commands[side][:6], grip]})

    def swing(self, phase, duration, side, pivot, axis, angle, orientation_fraction):
        feature, rotation = self.motion.feature(self.data, side)
        for amount in np.linspace(0, 1, round(self.pace*duration*RATE)+1)[1:]:
            progress = amount**2*(3.-2.*amount)
            turn = progress*angle*axis
            target = pivot + Rotation.from_rotvec(turn).apply(feature-pivot)
            orientation = Rotation.from_rotvec(orientation_fraction*turn).as_matrix() @ rotation
            self.commands[side] = np.r_[self.motion.solve(side, target, orientation,
                                                        self.commands[side][:6]), CLOSE]
            self._tick(phase)

    def relative(self, phase, side, delta, duration):
        initial, rotation = self.motion.feature(self.data, side)
        self.path(phase, duration, side, initial+delta, rotation, self.commands[side][6])

    def run(self, mode):
        self.hold("settle", 0.5)
        if mode == "hands_off":
            self.hold("hands_off", (ACTION_LIMIT-len(self.actions))/RATE)
            return
        if mode != "closed_press":
            self.vent(True)
            if mode == "vent":
                return
        self.remove_sleeve()
        if mode == "remove":
            return
        self.collapse()
        if mode in ("collapse", "closed_press"):
            self.jaw("release cup after folds", "right", OPEN, .65)
            self.relative("withdraw clear of cup", "right", np.array((0.,-.072,.010)), 1.0)
            self.joints("return right to rest", 1.25, right=np.r_[HOME,OPEN])
            self.hold("hands_clear", 6.)
            return
        self.put_sleeve_away()
        if mode == "leave_tab_open":
            self.hold("hands_clear", 2.45)
            return
        self.vent(False)
        self.joints("withdraw", 1.2, left=np.r_[HOME,OPEN])
        self.hold("hands_clear", 2.45)

    def vent(self, opening):
        center = LOCATION.copy()
        center[2] += 0.0 if opening else -.060
        left_pos = self.data.body("heat_sleeve").xpos.copy() + (0,0,.012)
        if opening:
            tab = center + (-.023,0.,.122)
            self.at("approach cup with both arms", 1.6,
                    left=(left_pos+(0,.12,.04), SLEEVE_GRIP, OPEN),
                    right=(tab+(0.,0.,.06), TAB_GRIP, .016))
            self.path("enter collar brace", 1., "left", left_pos, SLEEVE_GRIP, OPEN)
            self.jaw("brace collar with left fingers", "left", .013, .65)
            pivot = self.data.body("stopper").xpos.copy()
            tab = pivot + (-.028,0.,TAB_BIAS)
            self.path("straddle stopper", 1., "right", tab, TAB_GRIP, .014)
            self.jaw("pinch stopper", "right", CLOSE, .55)
            pivot = self.data.body("stopper").xpos.copy()
            self.swing("open vent", 1.35, "right", pivot, np.array((0, 1., 0)),
                       .59, ARC_FRACTION)
            self.jaw("release stopper", "right", .014,.3)
            self.path("clear open stopper", 1.1, "right", pivot+(-.06,-.09,.025), TAB_GRIP, .014)
        else:
            pivot = self.data.body("stopper").xpos.copy()
            tab_close = frame((.19,-.15,-1), (0,1,0))
            self.at("approach open stopper", 1.3, left=(pivot+(.035,.050,.086), tab_close, .014))
            self.path("straddle open stopper", 1.1, "left", pivot+CLOSURE_BIAS, tab_close, .014)
            self.jaw("grasp open stopper", "left", CLOSE, .45)
            pivot, stopper_orientation = self.object_pose("stopper")
            self.swing("close released stopper", 2.25, "left", pivot, stopper_orientation[:,1],
                       -1.38, CLOSURE_ARC_FRACTION)
            # Finish by pushing the partly rotated plug down into the drinking opening.
            at, orientation = self.motion.feature(self.data, "left")
            self.path("press plug home", 1.6, "left", at+PLUG_PRESS, orientation, CLOSE)
            self.jaw("let stopper seat", "left", .023,.48)
            _, orientation = self.motion.feature(self.data, "left")
            self.relative("back away along tool axis", "left", -.043*orientation[:,2], .95)

    def remove_sleeve(self):
        if self.commands["left"][6] != OPEN:
            self.jaw("release bracing hand", "left", OPEN, .45)
            self.relative("clear cup wall", "left", np.array((0,.075,0)), .75)
        sleeve = self.data.body("heat_sleeve").xpos.copy() + SLEEVE_OFFSET
        collar = self.data.body("stage_3").xpos.copy() + (0.,0.,.041)
        self.at("approach sleeve and collar", 1.6,
                left=(sleeve+(0,.075,.022), SLEEVE_GRIP, OPEN),
                right=(collar+(0.,-.095,.032), LID_GRIP, OPEN))
        self.path("enter sleeve low", 1., "left", sleeve, SLEEVE_GRIP, OPEN)
        self.jaw("grip sleeve low", "left", SLEEVE_CLOSE, .65)
        collar = self.data.body("stage_3").xpos.copy() + (0.,0.,.041)
        self.path("straddle collar", 1.25, "right", collar, LID_GRIP, OPEN)
        self.jaw("grip collar", "right", .014, .65)
        self.relative("pull sleeve down", "left", np.array((0,0,-PULL_DIST)), 1.4)
        self.relative("lift cup out of sleeve", "right", np.array((0,0,.087)), 2.0)
        self.jaw("secure separated sleeve", "left", .019, .55)
        self.relative("raise cup above sleeve", "right", np.array((0.,0.,.027)), 1.05)
        self.relative("move cup clear of sleeve", "right", np.array((.015,-.097,.008)), 1.6)
        self.relative("carry sleeve aside", "left", np.array((.010,.076,.003)), 1.7)
        self.jaw("set sleeve on table", "left", OPEN, .55)
        self.path("withdraw from free sleeve", 1.0, "left", np.array((.54,.150,TABLE_TOP+.065)),
                  frame((0,-1,-.25),(1,0,0)), OPEN)
        self.joints("clear parked sleeve", 1.4, left=np.r_[HOME,OPEN])

    def collapse(self):
        self.move_held("align cup upright", 1.4, "right", "cup_base",
                       (.532,-.047,TABLE_TOP+.070), np.eye(3))
        self.move_held("seat cup on table", 1.4, "right", "cup_base",
                       (.532,-.047,TABLE_TOP), np.eye(3))
        position, rotation = self.motion.feature(self.data,"right")
        final = position + np.array((0.,0.,-.062))
        self.path("fold bellows against table", 2.1, "right", final, rotation, .014)
        self.hold("settle bellows", .6)

    def object_pose(self, name):
        obj = self.data.body(name)
        return obj.xpos.copy(), obj.xmat.copy().reshape(3,3)

    def move_held(self, phase, duration, side, name, target, orientation, angular_tolerance=.052):
        old_p, old_r = self.object_pose(name)
        feat_p, feat_r = self.motion.feature(self.data, side)
        delta_r = orientation @ old_r.T
        endpoint = np.asarray(target) + delta_r @ (feat_p-old_p)
        self.path(phase, duration, side, endpoint, delta_r@feat_r, self.commands[side][6], angular_tolerance)

    def put_sleeve_away(self):
        self.move_held("raise compact cup", 1.4, "right", "cup_base",
                       (.527,-.045,TABLE_TOP+.083), np.eye(3))
        self.hold("settle parked sleeve", .35)
        sleeve_pos, sleeve_rotation = self.object_pose("heat_sleeve")
        if sleeve_rotation[2,2] < 0.:
            sleeve_pos += .022*sleeve_rotation[:,2]
        target = np.array((sleeve_pos[0], sleeve_pos[1], TABLE_TOP+.071))
        self.move_held("align cup above sleeve on table", 1.6, "right", "cup_base",
                       target, np.eye(3))
        target[2] = TABLE_TOP-.002
        self.move_held("lower cup directly into sleeve", 2.0, "right", "cup_base",
                       target, np.eye(3))
        self.hold("seat sleeve under collar", .65)
        self.jaw("set assembled cup on table", "right", OPEN, .65)
        self.relative("withdraw clear of packed cup", "right", np.array((0.,-.085,.010)), 1.0)
        self.joints("right arm clear for tab", 1.1, right=np.r_[HOME,OPEN])
        self.hold("packed cup stable on table", .6)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pace", type=float, default=DEFAULT_PACE,
                        help="Scale scripted segment durations before advancing physics")
    parser.add_argument("--mode", default="full",
                        choices=("vent","remove","collapse","full","hands_off","closed_press","leave_tab_open"))
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--play-commands", type=Path,
                        help="Replay saved joint targets from a fresh reset")
    args = parser.parse_args()
    trial = Trial(args.pace)
    args.out.mkdir(exist_ok=False,parents=True)
    frozen = args.out/"source"
    frozen.mkdir()
    for filename in ("scene.py","motion.py","run.py","scoring.py","render.py","review_media.py"):
        shutil.copy2(Path(__file__).parent/filename, frozen/filename)
    error = None
    try:
        if args.play_commands is not None:
            trial.replay(args.play_commands)
        else:
            trial.run(args.mode)
    except (ValueError,NotImplementedError) as exc:
        error = f"{type(exc).__name__}: {exc}"
    with (args.out/"trace.jsonl").open("w") as handle:
        for frame in trial.trace:
            handle.write(json.dumps(frame)+"\n")
    np.savez_compressed(args.out/"replay.npz",qpos=trial.states)
    np.savez_compressed(args.out/"commands.npz",actions=trial.actions)
    phases = {}
    for frame in trial.trace:
        phases[frame["phase"]] = frame
    report = {"error":error, "samples":len(trial.trace), "grade":grade(trial.trace),
              "control_hz": RATE, "physics_hz": RATE*SUBSTEPS, "action_limit": ACTION_LIMIT,
              "simulated_s": len(trial.trace)/RATE, "actuator_efforts": trial.effort_summary(),
              "endpoints":phases}
    (args.out/"summary.json").write_text(json.dumps(report,indent=2)+"\n")
    for name, row in phases.items():
        print(f"{name:23s} {row['stopper_deg']:7.2f}  {row['positions']['cup_base']} "
              f"{row['positions']['heat_sleeve']}  {row['folds_mm']}  {row['contact_n']}")
    if error:
        raise SystemExit(error)


if __name__ == "__main__":
    main()
