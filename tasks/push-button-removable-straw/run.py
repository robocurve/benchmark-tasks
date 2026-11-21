"""Develop calibrated joint targets and replay them from a fresh bottle reset."""
import argparse
import json
import os
from pathlib import Path
import time

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", "/usr/share/glvnd/egl_vendor.d/50_mesa.json")
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
os.environ.setdefault("MESA_SHADER_CACHE_DISABLE", "true")

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

from motion import Arm, ARMS, HOME, direction
from scene import Bottle, TABLE, build

CONTROL_HZ = 20
EFFORT = np.array((27.,27.,27.,7.,7.,7.))
ACTION_LIMIT = 750


def gripper_frame(yaw, pitch, roll=-90.):
    return direction(yaw,pitch) @ Rotation.from_euler("z",roll,degrees=True).as_matrix()


class Trial:
    def __init__(self, *, start_unlocked=True):
        self.m = mj.MjModel.from_xml_string(build())
        self.d = mj.MjData(self.m)
        self.physics = Bottle(self.m, self.d, unlocked_reset=start_unlocked)
        if start_unlocked:
            self.d.qpos[self.physics.ring] = 2.50
        self.arms=[Arm(self.m,side) for side in ARMS]
        self.plan=mj.MjData(self.m)
        self.goals=np.stack([HOME,HOME])
        self.orient=[None,None]
        self.phase="reset"
        self.trace=[]
        self.actions=[]
        self.peak=np.zeros((2,6))
        self.peak_fingers=np.zeros((2,2))
        self.peak_joint_speed=np.zeros((2,6))
        self.substeps=round(1/(CONTROL_HZ*self.m.opt.timestep))
        self.motion_errors=[]
        for arm in self.arms:
            self.m.actuator_forcerange[arm.actuators]=np.column_stack((-EFFORT,EFFORT))
            self.m.actuator_forcerange[arm.fingers]=(-40.,40.)
            arm.reset(self.d)
        mj.mj_forward(self.m,self.d)
        self.states=[self.d.qpos.copy()]

    def step(self, targets):
        if len(self.actions)>=ACTION_LIMIT:
            raise ValueError("action limit exceeded")
        targets = np.asarray(targets)
        if targets.shape != (2,7) or not np.isfinite(targets).all():
            raise ValueError("expected two finite six-joint targets and normalized jaw openings")
        if np.any((targets[:,6]<0.) | (targets[:,6]>1.)):
            raise ValueError("jaw opening must be between 0 and 1")
        for arm,goal in zip(self.arms,targets):
            arm.command(self.d,goal)
        for _ in range(self.substeps):
            self.physics.spring()
            mj.mj_step(self.m,self.d)
            self.physics.observe()
            for i,arm in enumerate(self.arms):
                self.peak[i]=np.maximum(self.peak[i],abs(self.d.actuator_force[arm.actuators]))
                self.peak_fingers[i]=np.maximum(self.peak_fingers[i],abs(self.d.actuator_force[arm.fingers]))
                self.peak_joint_speed[i]=np.maximum(self.peak_joint_speed[i],abs(self.d.qvel[arm.dofs]))
        self.goals=np.array(targets)
        self.actions.append(self.goals.copy())
        self.states.append(self.d.qpos.copy())
        row=self.physics.report()
        row.update(time_s=self.d.time,action=len(self.actions),phase=self.phase,
                   bottle_m=self.d.body("bottle").xpos.tolist(),
                   straw_m=self.d.body("straw").xpos.tolist(),
                   tcp_m=[self.tcp(i).tolist() for i in (0,1)])
        self.trace.append(row)

    def report(self):
        result = self.physics.report()
        return dict(result, actions=len(self.actions), time_s=self.d.time, events=self.physics.events,
                    peak_joint_effort_nm=self.peak.tolist(), peak_finger_effort_n=self.peak_fingers.tolist(),
                    peak_joint_speed_rads=self.peak_joint_speed.tolist(), motion_errors_m=self.motion_errors)

    def save_trace(self, path):
        with path.open("w") as trace:
            for row in self.trace:
                trace.write(json.dumps(row, allow_nan=False)+"\n")

    def ramp(self, targets, seconds, phase):
        self.phase=phase
        start=self.goals.copy()
        count=max(1,round(seconds*CONTROL_HZ))
        for k in range(1,count+1):
            t=k/count
            blend=t*t*(3-2*t)
            self.step(start+(targets-start)*blend)

    def move(self, side, target, duration, phase, opening=None, orientation=None):
        r=self.orient[side] if orientation is None else orientation
        q=self.arms[side].inverse(self.plan,np.asarray(target),r,self.goals[side,:6])
        destinations=self.goals.copy()
        destinations[side,:6]=q
        if opening is not None:
            destinations[side,6]=opening
        self.ramp(destinations,duration,phase)
        self.orient[side]=r
        error=float(np.linalg.norm(self.tcp(side)-target))
        self.motion_errors.append((phase,side,error))
        if duration>=.4:
            print(f"{phase}: TCP {self.tcp(side).round(3)} error {error*1000:.1f} mm",flush=True)

    def jaws(self, side, width, phase="adjust gripper"):
        dest=self.goals.copy()
        dest[side,6]=width
        self.ramp(dest,.55,phase)

    def tcp(self,side):
        return self.d.site_xpos[self.arms[side].site].copy()

    def xyz(self,name):
        return self.d.site(name).xpos.copy()

    def clear(self,side,phase):
        dest=self.goals.copy()
        dest[side]=HOME
        self.ramp(dest,1.65,phase)

    def brace(self,args):
        bottle=self.d.body("bottle")
        point=bottle.xpos + np.array((.0,args.brace_y,.061))
        r=gripper_frame(args.brace_yaw,args.brace_pitch)
        pre=point-r[:,2]*.105+[0,0,.024]
        self.move(0,pre,1.25,"approach bottle from the left",1.,r)
        self.move(0,point,.9,"straddle the bottle body",1.)
        self.jaws(0,args.brace_close,"hold the bottle body")
        print("Braced bottle at",self.d.body("bottle").xpos,flush=True)

    def press(self,args):
        position=self.xyz("button_contact_point")+np.asarray(args.press_offset)
        r=gripper_frame(args.press_yaw,args.press_pitch,args.press_roll)
        self.move(1,position+[0,.063,.02],1.0,"approach the push button",.018,r)
        self.move(1,position+[0,.017,0],.66,"line up with the push button",.018)
        for dy in (.008,.004,.001,-.002,-.004,-.006,-.008,-.010,-.012,-.014):
            self.move(1,position+[0,dy,0],.29,"press the spring catch")
            blocked=args.mode=="locked-press" and self.physics.last_ring_n>.50
            if blocked or self.physics.report()["lid_deg"]>45:
                break
        print("Button stage",self.physics.report(),flush=True)
        self.move(1,self.tcp(1)+[0,.070,.045],.85,"withdraw from the spring lid",.43)
        self.ramp(self.goals,.45,"let the straw rise")

    def pull(self,args):
        straw=self.d.body("straw")
        lifted_pos=straw.xpos.copy()
        start=self.xyz("straw_grip_site")+np.asarray(args.straw_offset)
        r=gripper_frame(args.straw_yaw,args.straw_pitch)
        self.move(1,start-r[:,2]*.069+[0,0,.016],1.18,"approach the exposed straw",.4,r)
        self.move(1,start,.68,"straddle exposed straw",.32)
        self.jaws(1,.015,"pinch the fitted straw")
        print("Straw grip",self.physics.report(),flush=True)
        hold_tcp=self.tcp(1)
        axis=self.d.body("bottle").xmat.reshape(3,3)[:,2].copy()
        for height in (.011,.023,.038,.058,.078,.099,.121,.142,.158,.172):
            self.move(1,hold_tcp+axis*height,.46,"pull straw straight out of the guide")
            if self.physics.extracted:
                break
        print("Pull stage",self.physics.report(),flush=True)
        if not self.physics.extracted:
            raise ValueError(f"fitted straw did not clear under grip, rose {straw.xpos[2]-lifted_pos[2]:.4f} m")
        self.move(1,self.tcp(1)+[-.139,.018,.004],1.25,"carry straw clear of the spout")
        if args.mode == "extract-only":
            return
        self.lay_straw(args)

    def lay_straw(self,args):
        r=self.d.site_xmat[self.arms[1].site].reshape(3,3).copy()
        self.move(1,self.tcp(1)+[0,0,-.06],.95,"lower the separated straw",orientation=r)
        r=self.d.site_xmat[self.arms[1].site].reshape(3,3).copy()
        at=self.tcp(1)
        offset=r.T@(self.d.body("straw").xpos-at)
        anchor=at+r@offset
        for angle in (20.,40.,60.,80.,90.):
            pose=r@Rotation.from_euler("z",angle*args.lay_turn,degrees=True).as_matrix()
            self.move(1,anchor-pose@offset,.38,"lay the detached straw horizontal",orientation=pose)
        lowest=min(self.xyz("straw_lower_end")[2], self.xyz("straw_grip_site")[2]-.008)
        self.move(1,self.tcp(1)+[0,0,TABLE+.001-lowest],1.35,
                  "set the straw beside the bottle")
        self.jaws(1,.37,"leave the straw on the table")
        self.move(1,self.tcp(1)+[0,.031,.057],.72,"leave the spout and straw clear",.54)

    def script(self,args):
        self.ramp(self.goals,.75,"tabletop reset")
        if args.mode=="hands-off":
            self.ramp(self.goals,(ACTION_LIMIT-len(self.actions))/CONTROL_HZ,"no manipulation")
            return
        self.brace(args)
        self.press(args)
        if args.mode not in ("open-only","locked-press"):
            if self.physics.report()["lid_deg"]<70:
                raise ValueError("lid did not open beyond 70 degrees")
            self.pull(args)
            if args.mode=="extract-only":
                self.ramp(self.goals,2.0,"retain the straw in the gripper")
                return
        self.clear(1,"return the right gripper to ready")
        self.jaws(0,1.,"release the bottle upright")
        self.move(0,self.tcp(0)+[.041,.060,.025],.85,"move away from the bottle",1.)
        self.clear(0,"return the left gripper to ready")
        self.ramp(self.goals,2.0,"retain wide-spout mode")


def run(args):
    started = time.monotonic()
    args.out.mkdir(parents=True)
    start_unlocked = args.mode != "locked-press"
    trial = Trial(start_unlocked=start_unlocked)
    error=None
    try:
        if args.replay:
            trial.phase="joint-only replay"
            for action in np.load(args.replay, allow_pickle=False):
                trial.step(action)
        else:
            trial.script(args)
    except ValueError as problem:
        error=str(problem)
        print(error,flush=True)
    trial.save_trace(args.out/"trace.jsonl")
    np.save(args.out/"actions.npy",np.asarray(trial.actions))
    np.savez_compressed(args.out/"states.npz",qpos=np.asarray(trial.states), final_qvel=trial.d.qvel)
    result=trial.report()
    result.update(status="complete" if not error else "motion failed",error=error,mode=args.mode,
                  start_unlocked=start_unlocked, replay=bool(args.replay),
                  wall_seconds=time.monotonic()-started)
    (args.out/"summary.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    print(json.dumps(result),flush=True)
    expected = {"full":1., "open-only":.30, "extract-only":.75, "hands-off":0., "locked-press":0.}
    if error or result["score"] != expected[args.mode]:
        raise SystemExit(1)


if __name__=="__main__":
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--out",required=True,type=Path)
    cli.add_argument("--replay",type=Path)
    cli.add_argument("--mode",choices=("full","open-only","extract-only","hands-off","locked-press"),default="full")
    cli.add_argument("--brace-y",type=float,default=.012)
    cli.add_argument("--brace-yaw",type=float,default=42.)
    cli.add_argument("--brace-pitch",type=float,default=24.)
    cli.add_argument("--brace-close",type=float,default=.61)
    cli.add_argument("--press-yaw",type=float,default=-18.)
    cli.add_argument("--press-pitch",type=float,default=27.)
    cli.add_argument("--press-roll",type=float,default=-90.)
    cli.add_argument("--press-offset",type=float,nargs=3,default=(0.,.003,.002))
    cli.add_argument("--straw-yaw",type=float,default=-25.)
    cli.add_argument("--straw-pitch",type=float,default=49.)
    cli.add_argument("--straw-offset",type=float,nargs=3,default=(0.,.003,.006))
    cli.add_argument("--lay-turn",type=float,default=1.)
    run(cli.parse_args())
